#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>

// MAVLINK Libraries
#include "mavlink/v2.0/rpsm_firmware_master/mavlink.h"


namespace rpsm_master {

class RpsmMainboard;

class FileManager {
 public:
  FileManager(ros::NodeHandle* nh, RpsmMainboard* rpsmMainboard);
  virtual ~FileManager();

  /// These methods are only used for testing purposes.
  bool sendCmdTestAck() { return sendOpcodeOnlyCmd(kCmdNone, kCOAck); };
  bool sendCmdTestNoAck() { return sendOpcodeOnlyCmd(kCmdTestNoAck, kCOAck); };

  /// Timeout in msecs to wait for an Ack time come back. This is public so we can write unit tests which wait long enough
  /// for the FileManager to timeout.
  static const int ackTimerTimeoutSecs = 100;

  /**
   * Downloads the specified file.
   * @param from File to download from UAS, fully qualified path
   * @param downloadDir Local directory to download file to
   */
  void downloadPath(const std::string& from, const std::string& downloadDir);

  /**
   * Stream downloads the specified file.
   * @param from File to download from RPSM, fully qualified path
   * @param downloadDir Local directory to download file to
   */
  void streamPath(const std::string& from, const std::string& downloadDir);

  /**
   * Lists the specified directory.
   * @param dirPath Fully qualified path to list
   * @return True if command was successfully sent
   */
  bool listDirectory(const std::string& dirPath);

  /**
   * Upload the specified file to the specified location
   */
  void uploadPath(const std::string& toPath, const std::string& uploadFile);

  /**
   * React to new message of type mavlink_file_transfer_protocol
   * @param message pure ftp mavlink message
   */
  void receiveMessage(mavlink_message_t* message);

  /**
   * Cancel a running session.
   */
  void preemptRunningSession();

 private:

  ros::NodeHandle* nh_;
  RpsmMainboard* rpsmMainboard_;

  struct RequestHeader
  {
      uint16_t    seqNumber;      ///< sequence number for message
      uint8_t     session;        ///< Session id for read and write commands
      uint8_t     opcode;         ///< Command opcode
      uint8_t     size;           ///< Size of data
      uint8_t     req_opcode;     ///< Request opcode returned in kRspAck, kRspNak message
      uint8_t     burstComplete;  ///< Only used if req_opcode=kCmdBurstReadFile - 1: set of burst packets complete, 0: More burst packets coming.
      uint8_t     padding;        ///< 32 bit aligment padding
      uint32_t    offset;         ///< Offsets for List and Read commands
  };

  struct Request
  {
      struct RequestHeader hdr;

      // We use a union here instead of just casting (uint32_t)&payload[0] to not break strict aliasing rules
      union {
          // The entire Request must fit into the payload member of the mavlink_file_transfer_protocol_t structure. We use as many leftover bytes
          // after we use up space for the RequestHeader for the data portion of the Request.
          uint8_t data[sizeof(((mavlink_file_transfer_protocol_t*)0)->payload) - sizeof(RequestHeader)];

          // File length returned by Open command
          uint32_t openFileLength;

          // Length of file chunk written by write command
          uint32_t writeFileLength;
      };
  };

  enum Opcode
  {
      kCmdNone,             ///< ignored, always acked
      kCmdTerminateSession, ///< Terminates open Read session
      kCmdResetSessions,    ///< Terminates all open Read sessions
      kCmdListDirectory,    ///< List files in <path> from <offset>
      kCmdOpenFileRO,       ///< Opens file at <path> for reading, returns <session>
      kCmdReadFile,         ///< Reads <size> bytes from <offset> in <session>
      kCmdCreateFile,       ///< Creates file at <path> for writing, returns <session>
      kCmdWriteFile,        ///< Writes <size> bytes to <offset> in <session>
      kCmdRemoveFile,       ///< Remove file at <path>
      kCmdCreateDirectory,  ///< Creates directory at <path>
      kCmdRemoveDirectory,  ///< Removes Directory at <path>, must be empty
      kCmdOpenFileWO,       ///< Opens file at <path> for writing, returns <session>
      kCmdTruncateFile,     ///< Truncate file at <path> to <offset> length
      kCmdRename,           ///< Rename <path1> to <path2>
      kCmdCalcFileCRC32,    ///< Calculate CRC32 for file at <path>
      kCmdBurstReadFile,    ///< Burst download session file

      kRspAck = 128,        ///< Ack response
      kRspNak,              ///< Nak response

      // Used for testing only, not part of protocol
      kCmdTestNoAck,        ///< ignored, ack not sent back, should timeout waiting for ack
  };

  /**
   * @brief Error codes returned in Nak response PayloadHeader.data[0].
   */
  enum ErrorCode
  {
      kErrNone,
      kErrFail,                   ///< Unknown failure
      kErrFailErrno,              ///< errno sent back in PayloadHeader.data[1]
      kErrInvalidDataSize,        ///< PayloadHeader.size is invalid
      kErrInvalidSession,         ///< Session is not currently open
      kErrNoSessionsAvailable,    ///< All available Sessions in use
      kErrEOF,                    ///< Offset past end of file for List and Read commands
      kErrUnknownCommand,         ///< Unknown command opcode
      kErrFailFileExists,         ///< File exists already
      kErrFailFileProtected       ///< File is write protected
  };

  enum OperationState
  {
      kCOIdle,      // not doing anything
      kCOAck,       // waiting for an Ack
      kCOList,      // waiting for List response
      kCOOpenRead,  // waiting for Open response followed by Read download
      kCOOpenBurst, // waiting for Open response, followed by Burst download
      kCORead,      // waiting for Read response
      kCOBurst,     // waiting for Burst response
      kCOWrite,     // waiting for Write response
      kCOCreate,    // waiting for Create response
  };

  void ackTimeout(const ros::TimerEvent &event);
  bool sendOpcodeOnlyCmd(uint8_t opcode, OperationState newOpState);
  void sendRequest(Request *request);
  void fillRequestWithString(Request *request, const std::string &str);
  void openAckResponse(Request *openAck);
  void downloadAckResponse(Request *readAck, bool readFile);
  void listAckResponse(Request *listAck);
  void createAckResponse(Request *createAck);
  void writeAckResponse(Request *writeAck);
  void writeFileDatablock();
  void sendListCommand();
  void sendResetCommand();
  void closeDownloadSession(bool success);
  void closeUploadSession(bool success);
  void downloadWorker(const std::string &from, const std::string &downloadDir, bool readFile);
  void completedList();

  static const std::string errorString(uint8_t errorCode);

  OperationState  currentOperation_;            ///< Current operation of state machine
  ros::Timer      ackTimer_;                    ///< Used to signal a timeout waiting for an ack

  uint16_t lastOutgoingSeqNumber_;              ///< Sequence number sent in last outgoing packet

  unsigned    listOffset_;                      ///< offset for the current List operation
  std::string listPath_;                        ///< path for the current List operation
  std::vector<std::string> listBuffer_;         ///< Holds list entries of current operation

  uint8_t     activeSession_;                   ///< currently active session, 0 for none

  uint32_t    readOffset_;                      ///< current read offset

  uint32_t    writeOffset_;                     ///< current write offset
  uint32_t    writeSize_;                       ///< current write data size
  uint32_t    writeFileSize_;                   ///< Size of file being uploaded
  std::vector<unsigned char> writeFileBuffer_;  ///< Holds file being uploaded

  uint32_t    downloadOffset_;                  ///< current download offset
  std::vector<unsigned char>  readFileBuffer_;  ///< Holds file being downloaded
  std::string        readFileDownloadDir_;      ///< Directory to download file to
  std::string     readFileDownloadFilename_;    ///< Filename (no path) for download file
  uint32_t    downloadFileSize_;                ///< Size of file being downloaded

  uint8_t     systemIdQGC_;                     ///< System ID for QGC
  uint8_t     systemIdServer_;                  ///< System ID for server
};

} // namespace rpsm_master

