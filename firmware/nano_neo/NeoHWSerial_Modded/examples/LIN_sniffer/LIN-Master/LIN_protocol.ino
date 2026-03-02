/**
  \fn byte LIN_protectID(byte id)
 
  \brief protect LIN ID with parity

  \param[in]    id        frame ID (protection optional)
  
  \return protected LIN identifier
 
  calculate protected LIN identifier as described in LIN2.0 spec "2.3.1.3 Protected identifier field"
*/
byte LIN_protectID(byte id) 
{
  byte          pid;
  uint8_t       tmp; // temporary variable use to calculate the parity bits

  // copy (unprotected) ID
  pid = id;
  
  // protect ID  with parity bits
  pid  = (uint8_t) (pid & 0x3F);    //0x3F =  00111111  =63
  tmp  = (uint8_t) ((pid ^ (pid>>1) ^ (pid>>2) ^ (pid>>4)) & 0x01);       // -> tmp[0] = PI0 =   ID0^ID1^ID2^ID4
  pid |= (uint8_t) (tmp<<6);
  tmp  = (uint8_t) (~((pid>>1) ^ (pid>>3) ^ (pid>>4) ^ (pid>>5)) & 0x01); // -> tmp[0] = PI1 =  ~(ID1^ID3^ID4^ID5)
  pid |= (uint8_t) (tmp<<7);
  
  // return protected identifier
  return(pid);
  
} // LIN_protectID



/**
  \fn byte LIN_sendMasterRequest(uint8_t vers, byte id, uint8_t numData, byte *data)
 
  \brief send LIN master request frame

  \param[in]    vers      LIN version (for checksum)
  \param[in]    id        frame ID (protection optional)
  \param[in]    numData   number of data bytes (0..8)
  \param[in]    data      Tx data bytes  (up to 8)

  \return error code
 
  send a LIN master request frame. For an explanation of the LIN bus
  and protocoll e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
*/
byte LIN_sendMasterRequest(uint8_t vers, byte id, uint8_t numData, byte *data)
{
  uint8_t       chk;        // tmp, 
  uint16_t      chk_tmp=0;
  byte          buf[11];      // frame Tx buffer: 0x55 + id + 8B data + chk
  
  // protect ID (see LIN2.0 spec "2.3.1.3 Protected identifier field")
  id  = LIN_protectID(id);
  
  // LIN2.x uses extended checksum which includes protected ID, i.e. including parity bits
  // LIN1.x uses classical checksum only over data bytes
  // Diagnostic frames with ID=0x3C/PID=0x3C and ID=0x3D/PID=0x7D always use classical checksum (see LIN2.0 spec "2.3.1.5 Checkum")
  if (!((vers == 1) || (id == 0x3C) || (id == 0x7D)))
    chk_tmp = (uint16_t) id;
  for (uint8_t i=0; i<numData; i++) {
    chk_tmp += (uint16_t) (data[i]);
    if (chk_tmp>255)
      chk_tmp -= 255;
  }
  chk = (uint8_t)(0xFF - ((uint8_t) chk_tmp));
   
  // construct Tx frame
  buf[0] = 0x55;                      // sync field
  buf[1] = id;                        // protected identifier
  for (uint8_t i=0; i<numData; i++)   // data bytes
    buf[2+i] = data[i];
  buf[2+numData] = chk;
    
  // empty receive buffer, just to be sure
  while(LIN_SERIAL.available())
    LIN_SERIAL.read();
  
  // generate sync break (low for >13bit @ baudrate)
  LIN_SERIAL.begin(LIN_BAUDRATE / 2);
  LIN_SERIAL.write(0x00);
  LIN_SERIAL.flush();
  LIN_SERIAL.begin(LIN_BAUDRATE);
      
  // send sync field + frame ID + data + checksum
  LIN_SERIAL.write(buf, 3+numData);
    
  // wait for the send is complete
  LIN_SERIAL.flush();
    
  // empty receive buffer, just to be sure
  while(LIN_SERIAL.available())
    LIN_SERIAL.read();

  // return error status
  return(SUCCESS);
    
} // LIN_sendMasterRequest



/**
  \fn byte LIN_receiveSlaveResponse(uint8_t vers, byte id, uint8_t numData, byte *data)
 
  \brief receive LIN slave response frame

  \param[in]    vers      LIN version 1 or 2 (for checksum)
  \param[in]    id        frame ID (protection optional)
  \param[in]    numData   number of data bytes (0..8)
  \param[out]   data      Rx data bytes (up to 8)

  \return error code
 
  receive a LIN slave response frame. For an explanation of the LIN bus
  and protocoll e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  
*/
byte LIN_receiveSlaveResponse(uint8_t vers, byte id, uint8_t numData, byte *data)
{
  uint8_t       tmp, chkCalc, numRx;     //chkRcv,
  uint16_t      chk_tmp=0;
  byte          buf[3];                 // frame Tx buffer: 0x55 + id
    
  // protect ID (see LIN2.0 spec "2.3.1.3 Protected identifier field")
  id  = LIN_protectID(id);
  
  // LIN2.x uses extended checksum which includes protected ID, i.e. including parity bits
  // LIN1.x uses classical checksum only over data bytes
  // Diagnostic frames with ID 0x3C and 0x3D always use classical checksum (see LIN spec "2.3.1.5 Checkum")
  if (!((vers == 1) || (id == 0x3C) || (id == 0x3D)))  //  3C  = 00111100  = 60; 3D  = 00111101  = 61
    chk_tmp = (uint16_t) id;
   
  // construct Tx frame
  buf[0] = 0x55;                      // sync field
  buf[1] = id;                        // protected identifier
    
  // clear Rx buffer
  for (uint8_t i=0; i<8; i++)
    data[i] = 0x00;

  // set timeout for data reception
  LIN_SERIAL.setTimeout(10); 
  
  // empty receive buffer, just to be sure
  while(LIN_SERIAL.available())
    LIN_SERIAL.read();
    
  // generate sync break (low for >13bit @ baudrate)
  LIN_SERIAL.begin(LIN_BAUDRATE / 2);
  LIN_SERIAL.write(0x00);
  LIN_SERIAL.flush();
  LIN_SERIAL.begin(LIN_BAUDRATE);

  // send sync field + frame ID
  LIN_SERIAL.write(buf, 2);
    
  // wait for the send is complete and clear receive buffer
  LIN_SERIAL.flush();
  
  // read 3B LIN echo (break+sync+id)
  for (tmp=0; tmp<3; tmp++)
    LIN_SERIAL.read();
  
  // receive data+chk with timeout
  numRx = LIN_SERIAL.readBytes(data, numData+1);

  
  // check for timeout
  if (numRx != numData+1) {

    // clear Rx buffer
    for (uint8_t i=0; i<8; i++)
      data[i] = 0x00;
    
    // return error code
    return(ERROR_TIMOUT);

  } // if timeout

  // calculate checksum over received bytes
  for (uint8_t i=0; i<numData; i++) {

    // update checksum
    chk_tmp += (uint16_t) (data[i]);
    if (chk_tmp>255)
      chk_tmp -= 255;

  } // checksum calculation

  // bitwise invert claculated checksum
  chkCalc = (uint8_t) (0xFF - ((uint8_t) chk_tmp));

  // compare calculated vs. received checksum
  if (data[numData] != chkCalc)
  {
    // clear Rx buffer
    for (uint8_t i=0; i<numData; i++)
      data[i] = 0x00;

    // return checksum error
    return(ERROR_CHK);

  } // if checksum error

  // return error status  
  return(SUCCESS);

} // LIN_receiveSlaveResponse       
