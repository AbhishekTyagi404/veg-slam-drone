#pragma once
struct Servo { int us=1500; bool att=false; int firstUs=-1;
  void attach(int,int,int){ att=true; if(firstUs<0) firstUs=us; }
  void writeMicroseconds(int v){ us=v; } };
