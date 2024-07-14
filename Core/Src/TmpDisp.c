#include "TmpDisp.h"

#define MAX(a, b) ((a) > (b) ? (a) : (b))

enum ModeTmpDispOnOff {
  ModeTmpDispOnOff_Off, ModeTmpDispOnOff_On
};

struct XfrpUnit xfrpUnit = {};

struct WithMode_ModeTmpDispOnOff_TupleDoubleDouble {
  enum ModeTmpDispOnOff mode[2];
  struct TupleDoubleDouble* value;
};

enum StateTmpDispMain_Tag {
  StateTmpDispMain_Tag_Show, StateTmpDispMain_Tag_Sleep
};
struct StateTmpDispMain {
  int mark;
  int fresh;
  enum StateTmpDispMain_Tag tag;
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
  struct WithMode_ModeTmpDispOnOff_TupleDoubleDouble* tmpHmdDisp;
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

struct WithMode_ModeTmpDispOnOff_TupleDoubleDouble tmpHmdDisp;

struct MemoryTmpDispMain memory;
int ModeTmpDispOnOff_is_accessible(enum ModeTmpDispOnOff modev);

static void mark_TupleDoubleDouble(struct TupleDoubleDouble*, int);
static void free_TupleDoubleDouble(struct TupleDoubleDouble*);
static struct StateTmpDispMain* StateTmpDispMain_Show(int);
static struct StateTmpDispMain* StateTmpDispMain_Sleep();
static void mark_StateTmpDispMain(struct StateTmpDispMain*, int);
static void free_StateTmpDispMain(struct StateTmpDispMain*);
static void header_init_TmpDispMain_state(struct MemoryTmpDispMain*);
static void update_TmpDispMainShow_tmpHmdDisp(struct MemoryTmpDispMain*);
static void update_TmpDispMainShow_state(struct MemoryTmpDispMain*);
static void update_TmpDispMainSleep_state(struct MemoryTmpDispMain*);
static void update_TmpDispMain(struct MemoryTmpDispMain*);
static void free_TmpDispMain(struct MemoryTmpDispMain*);
static enum ModeTmpDispOnOff TmpDispMain_calc_mode_tmpHmdDisp(struct MemoryTmpDispMain* memory);
static void refresh_mark();
extern struct TupleDoubleDouble* input_tmpHmd();
extern int input_show();
extern int input_clock();
extern void output_tmpHmdDisp(struct TupleDoubleDouble*);
extern void hook_tmpHmdDisp_ModeTmpDispOnOff_On_to_ModeTmpDispOnOff_Off();
extern void hook_tmpHmdDisp_ModeTmpDispOnOff_Off_to_ModeTmpDispOnOff_On();
int ModeTmpDispOnOff_is_accessible(enum ModeTmpDispOnOff modev) {
  return modev == ModeTmpDispOnOff_On;
}

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

static void mark_TupleDoubleDouble(struct TupleDoubleDouble* x, int mark) {
  if (mark > x->mark) { x->mark = mark; }
}

static void free_TupleDoubleDouble(struct TupleDoubleDouble* x) {
  x->mark = 0;
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
  x->tag = StateTmpDispMain_Tag_Show;
  x->params.Show.startClock = startClock;
  return x;
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
  x->tag = StateTmpDispMain_Tag_Sleep;
  return x;
}

static void mark_StateTmpDispMain(struct StateTmpDispMain* x, int mark) {
  if (mark > x->mark) { x->mark = mark; }
}

static void free_StateTmpDispMain(struct StateTmpDispMain* x) {
  x->mark = 0;
}

static void header_init_TmpDispMain_state(struct MemoryTmpDispMain* memory) {
  memory->state = StateTmpDispMain_Sleep();
}

static void update_TmpDispMainShow_tmpHmdDisp(struct MemoryTmpDispMain* memory) {
  memory->tmpHmdDisp->value = memory->tmpHmd[current_side];
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

static void update_TmpDispMainSleep_state(struct MemoryTmpDispMain* memory) {
  struct StateTmpDispMain* _tmpvar2;
  if (memory->show[current_side]) {
    _tmpvar2 = StateTmpDispMain_Show(memory->clock[current_side]);
  } else {
    _tmpvar2 = memory->state;
  }
  memory->state = _tmpvar2;
}

static void update_TmpDispMain(struct MemoryTmpDispMain* memory) {
  int entry = clock;
  if (memory->init) {
    header_init_TmpDispMain_state(memory);
    mark_StateTmpDispMain(memory->state, entry + 2);
  }
  clock = entry + 1;
  switch (memory->state->tag) {
    case StateTmpDispMain_Tag_Show: {
      if (memory->state->fresh) {
        memory->statebody.Show.init = 1;
      }
      memory->state->fresh = 0;
      mark_TupleDoubleDouble(memory->tmpHmd[current_side], entry + 3);
      mark_StateTmpDispMain(memory->state, entry + 4);
      clock = entry + 2;
      update_TmpDispMainShow_tmpHmdDisp(memory);
      if (ModeTmpDispOnOff_is_accessible(memory->tmpHmdDisp->mode[current_side])) {
        mark_TupleDoubleDouble(memory->tmpHmdDisp->value, entry + 5);
      }
      clock = entry + 3;
      update_TmpDispMainShow_state(memory);
      mark_StateTmpDispMain(memory->state, entry + clock + 1);
      clock = entry + 4;
      memory->statebody.Show.init = 0;
      break;
    }
    case StateTmpDispMain_Tag_Sleep: {
      if (memory->state->fresh) {
        memory->statebody.Sleep.init = 1;
      }
      memory->state->fresh = 0;
      mark_TupleDoubleDouble(memory->tmpHmd[current_side], entry + 2);
      mark_StateTmpDispMain(memory->state, entry + 3);
      clock = entry + 2;

      update_TmpDispMainSleep_state(memory);
      mark_StateTmpDispMain(memory->state, entry + clock + 1);
      clock = entry + 3;
      memory->statebody.Sleep.init = 0;
      break;
    }
  }
  memory->init = 0;
}

static void free_TmpDispMain(struct MemoryTmpDispMain* memory) {
  if (memory->init) return;
  if (memory->state->fresh) {
  }
  free_StateTmpDispMain(memory->state);
  memory->init = 1;
}

static enum ModeTmpDispOnOff TmpDispMain_calc_mode_tmpHmdDisp(struct MemoryTmpDispMain* memory) {
  if (memory->init) {
    return ModeTmpDispOnOff_Off;
  }
  switch (memory->state->tag) {
    case StateTmpDispMain_Tag_Show: {
      return ModeTmpDispOnOff_On;
      break;
    }
    case StateTmpDispMain_Tag_Sleep: {
      return ModeTmpDispOnOff_Off;
      break;
    }
  }
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
void activate() {
  current_side = 0;
  clock = 0;
  clock = 1;
  tmpHmdDisp.mode[!current_side] = ModeTmpDispOnOff_Off;
  memory.tmpHmdDisp = &tmpHmdDisp;
  memory.init = 1;
  while (1) {
    clock = 0;
    clock = 1;
    tmpHmdDisp.mode[current_side] = TmpDispMain_calc_mode_tmpHmdDisp(&memory);
    if (tmpHmdDisp.mode[!current_side] == ModeTmpDispOnOff_Off && tmpHmdDisp.mode[current_side] == ModeTmpDispOnOff_On) {
      hook_tmpHmdDisp_ModeTmpDispOnOff_Off_to_ModeTmpDispOnOff_On();
    }
    if (tmpHmdDisp.mode[!current_side] == ModeTmpDispOnOff_On && tmpHmdDisp.mode[current_side] == ModeTmpDispOnOff_Off) {
      hook_tmpHmdDisp_ModeTmpDispOnOff_On_to_ModeTmpDispOnOff_Off();
    }
    memory.clock[current_side] = input_clock();
    memory.show[current_side] = input_show();
    memory.tmpHmd[current_side] = input_tmpHmd();
    update_TmpDispMain(&memory);
    if (ModeTmpDispOnOff_is_accessible(tmpHmdDisp.mode[current_side])) {
      output_tmpHmdDisp(tmpHmdDisp.value);
    }
    clock = period;
    refresh_mark();
    current_side = !current_side;
  }
}
