#include "TmpDisp.h"

struct StateTmpDispMain {
  int mark;
  int fresh;
  int tag;
  union {
    struct {
      int startClock;
    } Show;
  } params;
};

struct MemoryTmpDispMainShow {
  int init;
};

struct MemoryTmpDispMainSleep {
  int init;
};

struct MemoryTmpDispMain {
  int init;
  int clock[2];
  int show[2];
  struct TupleDoubleDouble* tmpHmd[2];
  struct TupleDoubleDouble* tmpHmdDisp[2];
  struct StateTmpDispMain* state;
  union {
    struct MemoryTmpDispMainShow Show;
    struct MemoryTmpDispMainSleep Sleep;
  } statebody;
};

int clock;
int period = 6;
int current_side;

struct TupleDoubleDouble memory_TupleDoubleDouble[2];
int size_TupleDoubleDouble = 2;
int counter_TupleDoubleDouble = 0;

struct StateTmpDispMain memory_StateTmpDispMain[2];
int size_StateTmpDispMain = 2;
int counter_StateTmpDispMain = 0;

struct MemoryTmpDispMain memory;

static struct StateTmpDispMain* StateTmpDispMain_Show(int);
static struct StateTmpDispMain* StateTmpDispMain_Sleep();
static void mark_StateTmpDispMain(struct StateTmpDispMain*, int);
static void free_StateTmpDispMain(struct StateTmpDispMain*);
static void header_init_TmpDispMain_state(struct MemoryTmpDispMain*);
static void update_TmpDispMainShow_tmpHmdDisp(struct MemoryTmpDispMain*);
static void update_TmpDispMainShow_state(struct MemoryTmpDispMain*);
static void update_TmpDispMainSleep_tmpHmdDisp(struct MemoryTmpDispMain*);
static void update_TmpDispMainSleep_state(struct MemoryTmpDispMain*);
static void update_TmpDispMain(struct MemoryTmpDispMain*);
static void free_TmpDispMain(struct MemoryTmpDispMain*);
static void refresh_mark();
extern void input(int*, int*, struct TupleDoubleDouble**);
extern void output(struct TupleDoubleDouble**);

struct TupleDoubleDouble* TupleDoubleDouble_Cons(double member0, double member1) {
  struct TupleDoubleDouble* x;
  while (1) {
    counter_TupleDoubleDouble++;
    counter_TupleDoubleDouble %= size_TupleDoubleDouble;
    if (memory_TupleDoubleDouble[counter_TupleDoubleDouble].mark < clock) {
      x = memory_TupleDoubleDouble + counter_TupleDoubleDouble; break;
    }
  }
  x->mark = 0;
  x->member0 = member0;
  x->member1 = member1;
  return x;
}

void mark_TupleDoubleDouble(struct TupleDoubleDouble* x, int mark) {
  if (mark > x->mark) { x->mark = mark; }
}

void free_TupleDoubleDouble(struct TupleDoubleDouble* x) {
  x->mark = 0;
}

static void refresh_mark() {
  int i;
  for (i = 0; i < size_TupleDoubleDouble; ++i) {
    if (memory_TupleDoubleDouble[i].mark < period) memory_TupleDoubleDouble[i].mark = 0;
    else memory_TupleDoubleDouble[i].mark -= period;
  }
  for (i = 0; i < size_StateTmpDispMain; ++i) {
    if (memory_StateTmpDispMain[i].mark < period) memory_StateTmpDispMain[i].mark = 0;
    else memory_StateTmpDispMain[i].mark -= period;
  }
}

static void free_TmpDispMain(struct MemoryTmpDispMain* memory) {
  if (memory->init) return;
  if (memory->state->fresh) {
  }
  free_StateTmpDispMain(memory->state);
  memory->init = 1;
}

static void update_TmpDispMain(struct MemoryTmpDispMain* memory) {
  int entry = clock;
  if (memory->init) {
    header_init_TmpDispMain_state(memory);
    mark_StateTmpDispMain(memory->state, entry + 2);
  }
  clock = entry + 1;
  if (memory->state->tag == 0) {
    if (memory->state->fresh) {
      memory->statebody.Show.init = 1;
    }
    memory->state->fresh = 0;
    mark_TupleDoubleDouble(memory->tmpHmd[current_side], entry + 3);
    mark_StateTmpDispMain(memory->state, entry + 4);
    clock = entry + 2;
    update_TmpDispMainShow_tmpHmdDisp(memory);
    mark_TupleDoubleDouble(memory->tmpHmdDisp[current_side], entry + 5);
    clock = entry + 3;
    update_TmpDispMainShow_state(memory);
    mark_StateTmpDispMain(memory->state, entry + clock + 1);
    clock = entry + 4;
    memory->statebody.Show.init = 0;
  } else {
    if (memory->state->fresh) {
      memory->statebody.Sleep.init = 1;
    }
    memory->state->fresh = 0;
    mark_TupleDoubleDouble(memory->tmpHmd[current_side], entry + 2);
    mark_StateTmpDispMain(memory->state, entry + 4);
    clock = entry + 2;
    update_TmpDispMainSleep_tmpHmdDisp(memory);
    mark_TupleDoubleDouble(memory->tmpHmdDisp[current_side], entry + 5);
    clock = entry + 3;
    update_TmpDispMainSleep_state(memory);
    mark_StateTmpDispMain(memory->state, entry + clock + 1);
    clock = entry + 4;
    memory->statebody.Sleep.init = 0;
  }
  memory->init = 0;
}

static void update_TmpDispMainSleep_state(struct MemoryTmpDispMain* memory) {
  struct StateTmpDispMain* _tmpvar2;
  if (memory->show[current_side]) {
    _tmpvar2 = StateTmpDispMain_Show(memory->clock[current_side]);
  } else {
    _tmpvar2 = memory->state;
  }
  memory->state = _tmpvar2;
}

static void update_TmpDispMainSleep_tmpHmdDisp(struct MemoryTmpDispMain* memory) {
  memory->tmpHmdDisp[current_side] = TupleDoubleDouble_Cons(0., 0.);
}

static void update_TmpDispMainShow_state(struct MemoryTmpDispMain* memory) {
  struct StateTmpDispMain* _tmpvar1;
  if (((memory->clock[current_side] - memory->state->params.Show.startClock) >= 5)) {
    _tmpvar1 = StateTmpDispMain_Sleep();
  } else {
    _tmpvar1 = memory->state;
  }
  memory->state = _tmpvar1;
}

static void update_TmpDispMainShow_tmpHmdDisp(struct MemoryTmpDispMain* memory) {
  memory->tmpHmdDisp[current_side] = memory->tmpHmd[current_side];
}

static void header_init_TmpDispMain_state(struct MemoryTmpDispMain* memory) {
  memory->state = StateTmpDispMain_Sleep();
}

static void free_StateTmpDispMain(struct StateTmpDispMain* x) {
  x->mark = 0;
}

static void mark_StateTmpDispMain(struct StateTmpDispMain* x, int mark) {
  if (mark > x->mark) { x->mark = mark; }
}

static struct StateTmpDispMain* StateTmpDispMain_Sleep() {
  struct StateTmpDispMain* x;
  while (1) {
    counter_StateTmpDispMain++;
    counter_StateTmpDispMain %= size_StateTmpDispMain;
    if (memory_StateTmpDispMain[counter_StateTmpDispMain].mark < clock) {
      x = memory_StateTmpDispMain + counter_StateTmpDispMain; break;
    }
  }
  x->mark = 0;
  x->fresh = 1;
  x->tag = 1;
  return x;
}

static struct StateTmpDispMain* StateTmpDispMain_Show(int startClock) {
  struct StateTmpDispMain* x;
  while (1) {
    counter_StateTmpDispMain++;
    counter_StateTmpDispMain %= size_StateTmpDispMain;
    if (memory_StateTmpDispMain[counter_StateTmpDispMain].mark < clock) {
      x = memory_StateTmpDispMain + counter_StateTmpDispMain; break;
    }
  }
  x->mark = 0;
  x->fresh = 1;
  x->tag = 0;
  x->params.Show.startClock = startClock;
  return x;
}

void activate() {
  current_side = 0;
  clock = 0;
  clock = 1;
  memory.init = 1;
  while (1) {
    clock = 0;
    clock = 1;
    input(&memory.clock[current_side], &memory.show[current_side],
          &memory.tmpHmd[current_side]);
    update_TmpDispMain(&memory);
    output(&memory.tmpHmdDisp[current_side]);
    clock = period;
    refresh_mark();
    current_side = !current_side;
  }
}
