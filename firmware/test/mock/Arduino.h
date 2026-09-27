#pragma once
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <deque>
#include <cstdio>
typedef uint8_t byte;
#define F(x) (x)
#define RAD_TO_DEG 57.295779513082320876798154814105
using std::max; using std::min;
#include <algorithm>
extern unsigned long g_us;
inline unsigned long micros(){ return g_us; }
inline unsigned long millis(){ return g_us/1000; }
inline void delay(unsigned long ms){ g_us += ms*1000; }
struct MockSerial {
  std::deque<char> in; std::string out;
  void begin(long){}
  int available(){ return (int)in.size(); }
  int read(){ char c=in.front(); in.pop_front(); return c; }
  int availableForWrite(){ return 63; }
  void feed(const std::string&s){ for(char c:s) in.push_back(c); }
  void print(const char*s){ out+=s; } void print(char c){ out+=c; }
  void print(int v){ out+=std::to_string(v); } void print(unsigned long v){ out+=std::to_string(v); }
  void print(float v,int d){ char b[32]; snprintf(b,32,"%.*f",d,v); out+=b; }
  template<class T> void println(T v){ print(v); out+="\n"; }
  void println(float v,int d){ print(v,d); out+="\n"; }
};
extern MockSerial Serial;
