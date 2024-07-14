#ifndef TMPDISP_H
#define TMPDISP_H

struct XfrpUnit {};

struct TupleDoubleDouble {
  int mark;
  double member0;
  double member1;
};

struct TupleDoubleDouble* TupleDoubleDouble_Cons(double, double);

void activate();

#endif
