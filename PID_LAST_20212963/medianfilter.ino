#include "medianfilter.h"

namespace analog_reader_queue {
  volatile short queue[32];
  volatile byte writeIdx;
}

using namespace analog_reader_queue;

ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
  byte idx = writeIdx;
  queue[idx] = analogRead(MEDIANFILTER_IR_PIN);
  idx = (idx + 1) % 32;
  writeIdx = idx;
}

void swap(short& a, short& b) {
  short tmp = a;
  a = b;
  b = tmp;
}

void fixheap(short* arr, int node, int len) {
  while(node * 2 < len) {
    int leftChild = node * 2;
    int rightChild = node * 2 + 1;
    int largest = node;

    if(arr[largest] < arr[leftChild])
      largest = leftChild;
    if(rightChild < len && arr[largest] < arr[rightChild])
      largest = rightChild;

    if(largest != node) {
      swap(arr[largest], arr[node]);
      node = largest;
    }
    else
      break;
  }
}

void heapify(short* arr, int len) {
  for(int i = (len - 1) / 2; i >= 0; i--)
    fixheap(arr, i, len);
}

void heapremove(short* arr, int len) {
  arr[0] = arr[len - 1];
  fixheap(arr, 0, len - 1);
}
