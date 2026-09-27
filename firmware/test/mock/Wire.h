#pragma once
struct MockWire { bool timeoutFlag=false; void begin(){} void setClock(long){} void setWireTimeout(unsigned long,bool){}
  void clearWireTimeoutFlag(){} bool getWireTimeoutFlag(){ return timeoutFlag; } };
extern MockWire Wire;
