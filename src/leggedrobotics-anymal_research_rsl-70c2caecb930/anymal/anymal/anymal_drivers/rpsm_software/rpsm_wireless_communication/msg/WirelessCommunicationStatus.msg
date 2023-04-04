# Reports the number of valid and corrupt wirless packages recieved.
Header header           # timestamp in the header is the time the data is received
uint64 sentPacketCount  # total number of data packets sent
uint64 validPacketCount    # total number of valid data packets received
uint64 corruptPacketeCount # total number of corrupted data packets received