/*

Note Sequence Template

Note testSeq[16] = {
  quarterRest,
  quarterRest,
  quarterRest,
  quarterRest,
  quarterRest,
  quarterRest,
  quarterRest,
  quarterRest,
  quarterRest,
  quarterRest,
  quarterRest,
  quarterRest,
  quarterRest,
  quarterRest,
  quarterRest,
  quarterRest,
};


*/

// Note definitions
#define REST 0

#define A3 220
#define A4 440
#define A5 880

#define B4 494
#define C4 262
#define D4 294
#define E4 330
#define F4 349
#define G4 392

const uint8_t SLUT[64] = {
  128, 140, 153, 165, 177, 188, 199, 209, 
  219, 227, 235, 241, 246, 250, 253, 255,
  255, 254, 252, 248, 244, 238, 231, 223,
  214, 204, 194, 183, 171, 159, 147, 134,
  121, 108, 96, 84, 72, 61, 51, 41, 32,
  24, 17, 11, 7, 3, 1, 0, 0, 2, 5,
  9, 14, 20, 28, 36, 46, 56, 67, 78,
  90, 102, 115, 127
};

struct Note {
  Note(unsigned int _freq, unsigned long _dur){
    frequency = _freq;
    duration = _dur;
  }

  unsigned int frequency;
  unsigned long duration;
};

const Note quarterRest = Note(REST, 250);
const Note eigthRest = Note(REST, 125);
const Note testNote = Note(A4, 250);


Note testSeq[16] = {
  Note(C4, 250),
  quarterRest,
  Note(D4, 250),
  quarterRest,
  Note(E4, 250),
  quarterRest,
  Note(F4, 250),
  quarterRest,
  Note(G4, 250),
  quarterRest,
  Note(A4, 250),
  quarterRest,
  Note(B4, 250),
  quarterRest,
  quarterRest,
  quarterRest,
};

/*
  With a 64 bit LUT the phaseAcc must add to equal 64 in the period of a given note's frequency
  
  Therefore: phaseAcc = ((1 / frequency) / 64)
*/


class AudioManager {
  float phase, phaseAcc;
  unsigned int lastNoteChange = 0;
};






