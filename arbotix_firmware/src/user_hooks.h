/*
 * This file can be used to define custom register addresses
 * Just use addresses 100 and above
 */

void userSetup()
{
  // do any setup here
}

unsigned char userWrite(int addr, unsigned char param)
{
  // use the register value
  return ERR_INSTRUCTION;
}

/* Read one byte from register located at addr */
int userRead(int addr)
{
  // return the register value
  return 0; 
}
