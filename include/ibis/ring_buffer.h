#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_

#include <stdio.h>
#include <stdlib.h>

#include "management.h"

// リングバッファの初期化
inline RingBuffer * initRingBuffer(int size)
{
  RingBuffer * rb = (RingBuffer *)malloc(sizeof(RingBuffer));
  rb->buffer = (float *)malloc(size * sizeof(float));
  rb->size = size;
  rb->front = 0;
  rb->rear = -1;
  rb->count = 0;
  return rb;
}

// リングバッファに要素を追加
inline void enqueue(RingBuffer * rb, float data)
{
  if (rb->count < rb->size) {
    rb->rear = (rb->rear + 1) % rb->size;
    rb->buffer[rb->rear] = data;
    rb->count++;
  } else {
    // バッファがいっぱいの場合は古いデータを上書き
    rb->rear = (rb->rear + 1) % rb->size;
    rb->front = (rb->front + 1) % rb->size;
    rb->buffer[rb->rear] = data;
  }
}

// リングバッファから要素を取得
inline float dequeue(RingBuffer * rb)
{
  if (rb->count > 0) {
    float data = rb->buffer[rb->front];
    rb->front = (rb->front + 1) % rb->size;
    rb->count--;
    return data;
  } else {
    // バッファが空の場合は0.0を返すなど、適切なエラー処理を追加できます
    return 0.0;
  }
}

// リングバッファ上のデータを新しい順にn個加算した結果を取得
inline float sumNewestN(RingBuffer * rb, int n)
{
  if (n <= 0 || n > rb->count) {
    // 無効なnの値の場合はエラーとして0.0を返す
    return 0.0;
  }

  int index = rb->rear;
  float sum = 0.0;
  for (int i = 0; i < n; i++) {
    sum += rb->buffer[index];
    index = (index - 1 + rb->size) % rb->size;
  }
  return sum;
}

// リングバッファの解放
inline void freeRingBuffer(RingBuffer * rb)
{
  free(rb->buffer);
  free(rb);
}
#endif
