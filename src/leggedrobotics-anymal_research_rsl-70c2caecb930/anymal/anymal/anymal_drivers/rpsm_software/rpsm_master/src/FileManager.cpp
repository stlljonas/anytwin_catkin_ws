#include "rpsm_master/FileManager.hpp"

#include <fstream>

#include "rpsm_master/RpsmMainboard.hpp"


namespace rpsm_master {

FileManager::FileManager(ros::NodeHandle* nh, RpsmMainboard* rpsmMainboard)
    : nh_(nh)
    , rpsmMainboard_(rpsmMainboard)
    , currentOperation_(kCOIdle)
    , lastOutgoingSeqNumber_(0)
    , activeSession_(0)
    , systemIdQGC_(0u)
{
  nh_->createTimer(ros::Duration(ackTimerTimeoutSecs), &FileManager::ackTimeout, this);
  // TODO  systemIdServer_ = ?

  // Make sure we don't have bad structure packing
  assert(sizeof(RequestHeader) == 12);
}


FileManager::~FileManager() {

}


/**
 * Respond to the Ack associated with the Open command with the next read command.
 */
void FileManager::openAckResponse(Request* openAck) {
  ROS_INFO_STREAM("[RpsmMainboard] openAckResponse: currentOperation_(" << currentOperation_
                   << ") _readFileLength(" << openAck->openFileLength << ")");

  currentOperation_ = currentOperation_ == kCOOpenRead ? kCORead : kCOBurst;
  activeSession_ = openAck->hdr.session;

  // File length comes back in data
  assert(openAck->hdr.size == sizeof(uint32_t));
  downloadFileSize_ = openAck->openFileLength;

  // Start the sequence of read commands

  downloadOffset_ = 0;      // Start reading at beginning of file
  readFileBuffer_.clear();  // Start with an empty file

  Request request;
  request.hdr.session = activeSession_;
  assert(currentOperation_ == kCORead || currentOperation_ == kCOBurst);
  request.hdr.opcode = currentOperation_ == kCORead ? kCmdReadFile : kCmdBurstReadFile;
  request.hdr.offset = downloadOffset_;
  request.hdr.size = sizeof(request.data);

  sendRequest(&request);
}


/**
 * Closes out a download session by writing the file and doing cleanup.
 *  @param success true: successful download completion, false: error during download
 */
void FileManager::closeDownloadSession(bool success) {
  ROS_INFO("[RpsmMainboard] closeDownloadSession: success(%s)", success ? "true" : "false");

  currentOperation_ = kCOIdle;

  if (success) {
    std::ofstream file;
    std::string downloadFilePath;
    if (readFileDownloadDir_.find(".px4log") != std::string::npos ||
        readFileDownloadDir_.find(".txt") != std::string::npos)
    { 
      downloadFilePath = readFileDownloadDir_;
    } else {
      if (readFileDownloadDir_.back() != '/') {
        readFileDownloadDir_ += "/";
      }
      downloadFilePath = readFileDownloadDir_ + readFileDownloadFilename_;
    }
    file.open(downloadFilePath, std::ios::out | std::ios::binary);
    if (!file.is_open()) {
      ROS_ERROR("[RpsmMainboard] Unable to open local file for writing (%s)", downloadFilePath.c_str());
    }
    else {
      file.write((char*) readFileBuffer_.data(), std::streamsize(readFileBuffer_.size()));
      uint32_t bytesWritten = file.tellp();
      if (bytesWritten != std::streamsize(readFileBuffer_.size())) {
        ROS_ERROR("[RpsmMainboard] Unable to write data to local file (%s)", downloadFilePath.c_str());
      }
      file.close();
    }
  }

  readFileBuffer_.clear();

  // Close the open session
  sendResetCommand();
}


/**
 * Closes out an upload session doing cleanup.
 * @param success true: successful upload completion, false: error during download
 */
void FileManager::closeUploadSession(bool success) {
  ROS_INFO("[RpsmMainboard] closeUploadSession: success(%s)", success ? "true" : "false");

  currentOperation_ = kCOIdle;
  writeFileBuffer_.clear();
  writeFileSize_ = 0;

  if (success) {
    // TODO emit commandComplete(?);
  }

  // Close the open session
  sendResetCommand();
}


/**
 * Respond to the Ack associated with the Read or Stream commands.
 * @param readFile: true: read file, false: stream file
 */
void FileManager::downloadAckResponse(Request* readAck, bool readFile) {
  if (readAck->hdr.session != activeSession_) {
    closeDownloadSession(false /* failure */);
    ROS_ERROR("[RpsmMainboard] Download: Incorrect session returned");
    rpsmMainboard_->reactToFtpDownload(0.0, "Incorrect ftp session returned");
    return;
  }

  if (readAck->hdr.offset != downloadOffset_) {
    closeDownloadSession(false /* failure */);
    ROS_ERROR("[RpsmMainboard] Download: Offset returned (%i) differs from offset requested/expected (%i)",
              readAck->hdr.offset, downloadOffset_);
    rpsmMainboard_->reactToFtpDownload(0.0, "Offset returned differs from offset requested/expected");
    return;
  }

  ROS_DEBUG("[RpsmMainboard] downloadAckResponse: offset(%i) size(%i) readFile(%s) burstComplete(%s)",
            readAck->hdr.offset, readAck->hdr.size, readFile ? "true":"false",
            readAck->hdr.burstComplete != 0u ? "complete" : "more");

  downloadOffset_ += readAck->hdr.size;


  readFileBuffer_.insert(readFileBuffer_.end(), readAck->data,
                         readAck->data + readAck->hdr.size);

  if (downloadFileSize_ != 0) {
    float progress = ((float) readFileBuffer_.size()) / ((float) downloadFileSize_);
    ROS_INFO("[RpsmMainboard] %f %%", 100.0*progress);
    // Do nott set the action to succeeded just yet.
    if (progress == 1.0) progress -= 0.01;
    rpsmMainboard_->reactToFtpDownload(progress, std::string());
  }

  if (readFile || readAck->hdr.burstComplete != 0u) {
    // Possibly still more data to read, send next read request

    Request request;
    request.hdr.session = activeSession_;
    request.hdr.opcode = readFile ? kCmdReadFile : kCmdBurstReadFile;
    request.hdr.offset = downloadOffset_;
    request.hdr.size = 0;

    sendRequest(&request);
  } else if (!readFile) {
    // Streaming, so next ack should come automatically
    ackTimer_.setPeriod(ros::Duration(ackTimerTimeoutSecs), true /* reset */);
  }
}


/**
 * @brief Respond to the Ack associated with the List command.
 */
void FileManager::listAckResponse(Request* listAck) {
  if (listAck->hdr.offset != listOffset_) {
    currentOperation_ = kCOIdle;
    ROS_ERROR("[RpsmMainboard] List: Offset returned (%u) differs from offset requested (%u)",
              listAck->hdr.offset, listOffset_);
    return;
  }

  uint8_t offset = 0;
  uint8_t cListEntries = 0;
  uint8_t cBytes = listAck->hdr.size;

  // parse filenames out of the buffer
  while (offset < cBytes) {
    const char* ptr = ((const char*) listAck->data) + offset;

    // get the length of the name
    uint8_t cBytesLeft = cBytes - offset;
    uint8_t nlen = static_cast<uint8_t>(strnlen(ptr, cBytesLeft));
    if ((*ptr == 'S' && nlen > 1) || (*ptr != 'S' && nlen < 2)) {
      currentOperation_ = kCOIdle;
      ROS_ERROR("[RpsmMainboard] Incorrectly formed list entry: '%c'", *ptr);
      return;
    } else if (nlen == cBytesLeft) {
      currentOperation_ = kCOIdle;
      ROS_ERROR("[RpsmMainboard] Missing NULL termination in list entry");
      return;
    }

    // Returned names are prepended with D for directory, F for file, S for skip
    ROS_DEBUG_STREAM("[RpsmMainboard] List entry " << std::string(ptr));
    if (*ptr == 'F' || *ptr == 'D') {
      // put it in the view
      listBuffer_.push_back(std::string(ptr));
    } else if (*ptr == 'S') {
      // do nothing
    } else {
      ROS_WARN_STREAM("[RpsmMainboard] unknown entry" << *ptr);
    }

    // account for the name + NUL
    offset += nlen + 1;

    cListEntries++;
  }

  if (listAck->hdr.size == 0 || cListEntries == 0) {
    // Directory is empty, we're done
    assert(listAck->hdr.opcode == kRspAck);
    currentOperation_ = kCOIdle;
    completedList();
  } else {
    // Possibly more entries to come, need to keep trying till we get EOF
    currentOperation_ = kCOList;
    listOffset_ += cListEntries;
    sendListCommand();
  }
}


/**
 * @brief Respond to the Ack associated with the create command.
 */
void FileManager::createAckResponse(Request* createAck) {
  ROS_DEBUG("[RpsmMainboard] createAckResponse");

  currentOperation_ = kCOWrite;
  activeSession_ = createAck->hdr.session;

  // Start the sequence of write commands from the beginning of the file

  writeOffset_ = 0;
  writeSize_ = 0;

  writeFileDatablock();
}


/**
 * @brief Respond to the Ack associated with the write command.
 */
void FileManager::writeAckResponse(Request* writeAck) {
  if (writeOffset_ + writeSize_ >= writeFileSize_) {
    closeUploadSession(true /* success */);
  }

  if (writeAck->hdr.session != activeSession_) {
    closeUploadSession(false /* failure */);
    ROS_ERROR("[RpsmMainboard] Write: Incorrect session returned");
    return;
  }

  if (writeAck->hdr.offset != writeOffset_) {
    closeUploadSession(false /* failure */);
    ROS_ERROR("[RpsmMainboard] Write: Offset returned (%u) differs from offset requested (%u)",
              writeAck->hdr.offset, writeOffset_);
    return;
  }

  if (writeAck->hdr.size != sizeof(uint32_t)) {
    closeUploadSession(false /* failure */);
    ROS_ERROR("[RpsmMainboard] Write: Returned invalid size of write size data");
    return;
  }


  if (writeAck->writeFileLength != writeSize_) {
    closeUploadSession(false /* failure */);
    ROS_ERROR("[RpsmMainboard] Write: Size returned (%u) differs from size requested (%u)",
              writeAck->writeFileLength, writeSize_);
    return;
  }

  writeFileDatablock();
}


/**
 * @brief Send next write file data block.
 */
void FileManager::writeFileDatablock(void) {
  if (writeOffset_ + writeSize_ >= writeFileSize_) {
    closeUploadSession(true /* success */);
    return;
  }

  writeOffset_ += writeSize_;

  Request request;
  request.hdr.session = activeSession_;
  request.hdr.opcode = kCmdWriteFile;
  request.hdr.offset = writeOffset_;

  if (writeFileSize_ - writeOffset_ > sizeof(request.data))
    writeSize_ = sizeof(request.data);
  else
    writeSize_ = writeFileSize_ - writeOffset_;

  request.hdr.size = writeSize_;

  memcpy(request.data, &(writeFileBuffer_[writeOffset_]), writeSize_);

  sendRequest(&request);
}


void FileManager::receiveMessage(mavlink_message_t* message) {

  mavlink_file_transfer_protocol_t data;
  mavlink_msg_file_transfer_protocol_decode(message, &data);

  // Make sure we are the target system
//  if (data.target_system != systemIdQGC_) {
//    ROS_DEBUG_STREAM("[RpsmMainboard] Received MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL with incorrect target_system: "
//                     << data.target_system << "expected: " << systemIdQGC_ );
//  }

  Request* request = (Request *) &data.payload[0];

  ackTimer_.stop();

  uint16_t incomingSeqNumber = request->hdr.seqNumber;


  ROS_DEBUG_STREAM("[RpsmMainboard] receiveMessage(opcode: " << (unsigned int) request->hdr.opcode << ", seqNumber: " << incomingSeqNumber << ")");

  // Make sure we have a good sequence number
  uint16_t expectedSeqNumber = lastOutgoingSeqNumber_ + 1u;
  if (incomingSeqNumber != expectedSeqNumber) {
    switch (currentOperation_) {
      case kCOBurst:
      case kCORead:
        closeDownloadSession(false /* failure */);
        rpsmMainboard_->reactToFtpDownload(0.0, "Received unexpected sequence number");
        break;

      case kCOWrite:
        closeUploadSession(false /* failure */);
        break;

      case kCOOpenRead:
      case kCOOpenBurst:
      case kCOCreate:
        // We could have an open session hanging around
        currentOperation_ = kCOIdle;
        sendResetCommand();
        break;

      default:
        // Don't need to do anything special
        currentOperation_ = kCOIdle;
        break;
    }

    ROS_ERROR("[RpsmMainboard] Bad sequence number on received message: expected(%u) received(%u)",
              expectedSeqNumber, incomingSeqNumber);
    return;
  }

  // Move past the incoming sequence number for next request
  lastOutgoingSeqNumber_ = incomingSeqNumber;

  if (request->hdr.opcode == kRspAck) {
    switch (request->hdr.req_opcode) {
      case kCmdListDirectory:
        listAckResponse(request);
        break;

      case kCmdOpenFileRO:
      case kCmdOpenFileWO:
        openAckResponse(request);
        break;

      case kCmdReadFile:
        downloadAckResponse(request, true /* read file */);
        break;

      case kCmdBurstReadFile:
        downloadAckResponse(request, false /* stream file */);
        break;

      case kCmdCreateFile:
        createAckResponse(request);
        break;

      case kCmdWriteFile:
        writeAckResponse(request);
        break;

      default:
        // Ack back from operation which does not require additional work
        currentOperation_ = kCOIdle;
        break;
    }
  } else if (request->hdr.opcode == kRspNak) {
    uint8_t errorCode = request->data[0];

    // Nak's normally have 1 byte of data for error code, except for kErrFailErrno which has additional byte for errno
    assert((errorCode == kErrFailErrno && request->hdr.size == 2) || request->hdr.size == 1);

    currentOperation_ = kCOIdle;

    if (request->hdr.req_opcode == kCmdListDirectory && errorCode == kErrEOF) {
      // This is not an error, just the end of the list loop
      completedList();
      return;
    } else if ((request->hdr.req_opcode == kCmdReadFile || request->hdr.req_opcode == kCmdBurstReadFile) &&
               errorCode == kErrEOF) {
      // This is not an error, just the end of the download loop
      closeDownloadSession(true /* success */);
      rpsmMainboard_->reactToFtpDownload(1.0, std::string());
      return;
    } else if (request->hdr.req_opcode == kCmdCreateFile) {
      ROS_ERROR_STREAM("[RpsmMainboard] Nak received creating file, error: " << errorString(request->data[0]));
      return;
    } else {
      // Generic Nak handling
      if (request->hdr.req_opcode == kCmdReadFile || request->hdr.req_opcode == kCmdBurstReadFile) {
        // Nak error during download loop, download failed
        closeDownloadSession(false /* failure */);
        rpsmMainboard_->reactToFtpDownload(0.0, "Nak error during download loop");
      } else if (request->hdr.req_opcode == kCmdWriteFile) {
        // Nak error during upload loop, upload failed
        closeUploadSession(false /* failure */);
      }
      ROS_ERROR_STREAM("[RpsmMainboard] Nak received, error: " << errorString(request->data[0]));
    }
  } else {
    // Note that we don't change our operation state. If something goes wrong beyond this, the operation
    // will time out.
    ROS_ERROR("[RpsmMainboard] Unknown opcode returned from server: %u", request->hdr.opcode);
  }
}


void FileManager::preemptRunningSession() {
  switch (currentOperation_) {
    case kCORead:
    case kCOBurst:
      closeDownloadSession(false /* failure */);
      break;
    case kCOOpenRead:
    case kCOOpenBurst:
      currentOperation_ = kCOIdle;
      sendResetCommand();
      break;
    case kCOCreate:
      currentOperation_ = kCOIdle;
      sendResetCommand();
      break;
    case kCOWrite:
      closeUploadSession(false /* failure */);
      break;
    default:
      currentOperation_ = kCOIdle;
      break;
  }
}


bool FileManager::listDirectory(const std::string& dirPath) {
  if (currentOperation_ != kCOIdle) {
    ROS_ERROR("[RpsmMainboard] Command not sent. Waiting for previous command to complete.");
    return false;
  }

  // initialise the lister
  listPath_ = dirPath;
  listOffset_ = 0;
  currentOperation_ = kCOList;

  // reset list
  listBuffer_.clear();


  // and send the initial request
  sendListCommand();
  return true;
}


void FileManager::fillRequestWithString(Request* request, const std::string &str) {
  strncpy((char*) &request->data[0], str.c_str(), sizeof(request->data));
  request->hdr.size = static_cast<uint8_t>(strnlen((const char*) &request->data[0], sizeof(request->data)));
}


void FileManager::sendListCommand(void) {
  Request request;

  request.hdr.session = 0;
  request.hdr.opcode = kCmdListDirectory;
  request.hdr.offset = listOffset_;
  request.hdr.size = 0;

  fillRequestWithString(&request, listPath_);

  ROS_INFO_STREAM("[RpsmMainboard] sendListCommand(): path: '" << listPath_ << "' offset: " << listOffset_);

  sendRequest(&request);
}


void FileManager::downloadPath(const std::string& from, const std::string& downloadDir) {
  if (currentOperation_ != kCOIdle) {
    ROS_ERROR("[RpsmMainboard] Command not sent. Waiting for previous command to complete.");
    return;
  }

  ROS_INFO_STREAM("[RpsmMainboard] downloadPath from:" << from << "to:" << downloadDir);
  downloadWorker(from, downloadDir, true /* read file */);
}


void FileManager::streamPath(const std::string& from, const std::string& downloadDir) {
  if (currentOperation_ != kCOIdle) {
    ROS_ERROR("[RpsmMainboard] Command not sent. Waiting for previous command to complete.");
    return;
  }

  ROS_INFO_STREAM("[RpsmMainboard] streamPath from: [rpsm]" << from << " to: [LPC]" << downloadDir);
  downloadWorker(from, downloadDir, false /* stream file */);
}


void FileManager::downloadWorker(const std::string &from, const std::string &downloadDir, bool readFile) {
  if (from.empty()) {
	ROS_ERROR("[RpsmMainboard] No stream path specified.");
    return;
  }

  readFileDownloadDir_ = downloadDir;

  // We need to strip off the file name from the fully qualified path.

  std::size_t found = from.find_last_of("/\\");
  readFileDownloadFilename_ = from.substr(found + 1);

  currentOperation_ = readFile ? kCOOpenRead : kCOOpenBurst;

  Request request;
  request.hdr.session = 0;
  request.hdr.opcode = kCmdOpenFileRO;
  request.hdr.offset = 0;
  request.hdr.size = 0;
  fillRequestWithString(&request, from);
  sendRequest(&request);
}


/**
 * @brief Uploads the specified file.
 * @param toPath File in UAS to upload to, fully qualified path
 * @param uploadFile Local file to upload from
 */
void FileManager::uploadPath(const std::string& toPath, const std::string& uploadFile) {
  if (currentOperation_ != kCOIdle) {
    ROS_ERROR("[RpsmMainboard] File manager busy.  Try again later");
    return;
  }

  if (toPath.empty()) {
    return;
  }

  std::ifstream file;
  file.open(uploadFile, std::ios::in | std::ios::binary);
  if (!file.good()) {
    ROS_ERROR("[RpsmMainboard] File (%s) is not readable for upload", uploadFile.c_str());
    return;
  }

  if (!file.is_open()) {
    ROS_ERROR("[RpsmMainboard] Unable to open local file for upload (%s)", uploadFile.c_str());
    return;
  }

  file.seekg(0, std::ios_base::end);
  std::streampos fileSize = file.tellg();
  writeFileBuffer_.resize(fileSize);

  file.seekg(0, std::ios_base::beg);
  file.read((char*) (&writeFileBuffer_[0]), fileSize);
  file.close();

  if (writeFileBuffer_.size() == 0) {
    ROS_ERROR("[RpsmMainboard] Unable to read data from local file (%s)", uploadFile.c_str());
    return;
  }

  currentOperation_ = kCOCreate;

  Request request;
  request.hdr.session = 0;
  request.hdr.opcode = kCmdCreateFile;
  request.hdr.offset = 0;
  request.hdr.size = 0;
  fillRequestWithString(&request, toPath + "/" + uploadFile);
  sendRequest(&request);
}

const std::string FileManager::errorString(uint8_t errorCode) {
  switch (errorCode) {
    case kErrNone:
      return "no error";
    case kErrFail:
      return "unknown error";
    case kErrEOF:
      return "read beyond end of file";
    case kErrUnknownCommand:
      return "unknown command";
    case kErrFailErrno:
      return "command failed";
    case kErrInvalidDataSize:
      return "invalid data size";
    case kErrInvalidSession:
      return "invalid session";
    case kErrNoSessionsAvailable:
      return "no sessions available";
    case kErrFailFileExists:
      return "File already exists on target";
    case kErrFailFileProtected:
      return "File is write protected";
    default:
      return "unknown error code";
  }
}


/**
 * @brief Sends a command which only requires an opcode and no additional data
 * @param opcode Opcode to send
 * @param newOpState State to put state machine into
 * @return TRUE: command sent, FALSE: command not sent, waiting for previous command to finish
 */
bool FileManager::sendOpcodeOnlyCmd(uint8_t opcode, OperationState newOpState) {
  if (currentOperation_ != kCOIdle) {
    // Can't have multiple commands in play at the same time
    return false;
  }

  Request request;
  request.hdr.session = 0;
  request.hdr.opcode = opcode;
  request.hdr.offset = 0;
  request.hdr.size = 0;

  currentOperation_ = newOpState;

  sendRequest(&request);

  return true;
}


/**
 * @brief Called when ack timeout timer fires
 */
void FileManager::ackTimeout(const ros::TimerEvent &event) {
  ROS_WARN_STREAM("[RpsmMainboard] ackTimeout");

  // Make sure to set currentOperation_ state before emitting error message. Code may respond
  // to error message signal by sending another command, which will fail if state is not back
  // to idle. FileView UI works this way with the List command.

  switch (currentOperation_) {
    case kCORead:
    case kCOBurst:
      closeDownloadSession(false /* failure */);
      ROS_ERROR("[RpsmMainboard] Timeout waiting for ack: Download failed");
      rpsmMainboard_->reactToFtpDownload(0.0, "Timeout waiting for ack");
      break;

    case kCOOpenRead:
    case kCOOpenBurst:
      currentOperation_ = kCOIdle;
      ROS_ERROR("[RpsmMainboard] Timeout waiting for ack: Download failed");
      sendResetCommand();
      break;

    case kCOCreate:
      currentOperation_ = kCOIdle;
      ROS_ERROR("[RpsmMainboard] Timeout waiting for ack: Upload failed");
      sendResetCommand();
      break;

    case kCOWrite:
      closeUploadSession(false /* failure */);
      ROS_ERROR("[RpsmMainboard] Timeout waiting for ack: Upload failed");
      break;

    default:
      currentOperation_ = kCOIdle;
      ROS_ERROR("[RpsmMainboard] Timeout waiting for ack: Command failed (%i)", currentOperation_);
      break;
  }
}


void FileManager::sendResetCommand(void) {
  Request request;
  request.hdr.opcode = kCmdResetSessions;
  request.hdr.size = 0;
  sendRequest(&request);
}


/**
 * @brief Sends the specified Request.
 */
void FileManager::sendRequest(Request* request) {

  ackTimer_.setPeriod(ros::Duration(ackTimerTimeoutSecs), true /* reset */);

  lastOutgoingSeqNumber_++;

  request->hdr.seqNumber = lastOutgoingSeqNumber_;

  ROS_DEBUG_STREAM("[RpsmMainboard] sendRequest opcode: " << (unsigned int) request->hdr.opcode << " seqNumber: "
                          << request->hdr.seqNumber);

  rpsmMainboard_->sendFtpMessage((uint8_t*) request);
}


void FileManager::completedList() {
  rpsmMainboard_->reactToFtpList(listBuffer_);
}

} // namespace rpsm_master
