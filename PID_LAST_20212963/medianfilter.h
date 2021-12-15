#ifndef INTERPOLATE_MODEL_H_
#define INTERPOLATE_MODEL_H_


#include "Arduino.h"


#ifndef MEDIANFILTER_IR_PIN
  #if defined(PIN_IR)
    #define MEDIANFILTER_IR_PIN PIN_IR
  #elif defined(IR_PIN)
    #define MEDIANFILTER_IR_PIN IR_PIN
  #else
    #error Please define either IR_PIN -OR- PIN_IR (e.g. A0)
  #endif
#endif


namespace analog_reader_queue {
  extern volatile short queue[32];
  extern volatile byte writeIdx;
}


void swap(short& a, short& b);
void fixheap(short* arr, int node, int len);
void heapify(short* arr, int len);
void heapremove(short* arr, int len);


template<float(*fn)(short)>
struct TypeDetector {
  typedef float Type;
};

template<>
struct TypeDetector<nullptr> {
  typedef short Type;
};


template<float(*fnCalcDistance)(short) = nullptr>
class MedianFilter {
public:
  void init() {
    analogRead(MEDIANFILTER_IR_PIN);

    cli();

    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS22) | (1 << CS21);
    OCR2A = 150;
    TIMSK2 = (1 << OCIE2A);

    sei();
  }

  bool ready() {
    byte writeIdx = analog_reader_queue::writeIdx;
    if(writeIdx < readIdx)
      writeIdx += 32;
    return readIdx + 16 < writeIdx;
  }

  typename TypeDetector<fnCalcDistance>::Type read() {
    short arr[16];
    for(int i=0; i<16; i++) {
      arr[i] = analog_reader_queue::queue[readIdx];
      readIdx = (readIdx + 1) % 32;
    }

    int len = 16;
    heapify(arr, len);

    for(int i=0; i<7; i++)
      heapremove(arr, len--);

    short rightMedian = arr[0];
    if(fnCalcDistance != nullptr) {
      heapremove(arr, len--);
      short leftMedian = arr[0];
      return (fnCalcDistance(rightMedian) + fnCalcDistance(leftMedian)) * 0.5f;
    }
    else
      return rightMedian;
  }

private:
  byte readIdx;
};


#endif
