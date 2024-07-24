#ifndef TMPDISP_H
#define TMPDISP_H

struct TupleDoubleDouble {
  int mark;
  double member0;
  double member1;
};



struct TupleDoubleDouble* TupleDoubleDouble_Cons(double, double);
void mark_TupleDoubleDouble(struct TupleDoubleDouble*, int);
void free_TupleDoubleDouble(struct TupleDoubleDouble*);

void activate();

#endif
