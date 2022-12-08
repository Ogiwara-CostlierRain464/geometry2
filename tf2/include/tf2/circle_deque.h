#ifndef GEOMETRY2_CIRCLE_DEQUE_H
#define GEOMETRY2_CIRCLE_DEQUE_H

#include <iostream>
#include <atomic>
using namespace std;

// Maximum size of array or Dequeue
#define MAX 100

template <class T>
class Deque{
public:
  T arr[MAX];
private:
  atomic_int front;
  atomic_int rear;
  atomic_int size;

public:
  Deque(int size)
  {
    front = -1;
    rear = 0;
    this->size = size;
  }

  // Operations on Deque:
  void insertFront(T key);
  void insertRear(T key);
  void deleteFront();
  void deleteRear();
  bool isFull();
  bool isEmpty();
  int getFront();
  int getRear();
};

// Checks whether Deque is full or not.
template <class T>
bool Deque<T>::isFull()
{
  return ((front == 0 && rear == size - 1)
          || front == rear + 1);
}

template <class T>
bool Deque<T>::isEmpty() { return (front == -1); }

template <class T>
void Deque<T>::insertFront(T key)
{
  // check whether Deque if full or not
  if (isFull()) {
    fprintf(stderr, "Overflow\n");
    return;
  }

  // If queue is initially empty
  if (front == -1) {
    front = 0;
    rear = 0;
  }

    // front is at first position of queue
  else if (front == 0)
    front = size - 1;

  else // decrement front end by '1'
    front = front - 1;

  // insert current element into Deque
  arr[front] = key;
}

template <class T>
void Deque<T> ::insertRear(T key)
{
  if (isFull()) {
    cout << " Overflow\n " << endl;
    return;
  }

  // If queue is initially empty
  if (front == -1) {
    front = 0;
    rear = 0;
  }

    // rear is at last position of queue
  else if (rear == size - 1)
    rear = 0;

    // increment rear end by '1'
  else
    rear = rear + 1;

  // insert current element into Deque
  arr[rear] = key;
}

// Deletes element at front end of Deque
template <class T>
void Deque<T>::deleteFront()
{
  // check whether Deque is empty or not
  if (isEmpty()) {
    cout << "Queue Underflow\n" << endl;
    return;
  }

  // Deque has only one element
  if (front == rear) {
    front = -1;
    rear = -1;
  }
  else
    // back to initial position
  if (front == size - 1)
    front = 0;

  else // increment front by '1' to remove current
    // front value from Deque
    front = front + 1;
}

template <class T>
void Deque<T>::deleteRear()
{
  if (isEmpty()) {
    cout << " Underflow\n" << endl;
    return;
  }

  // Deque has only one element
  if (front == rear) {
    front = -1;
    rear = -1;
  }
  else if (rear == 0)
    rear = size - 1;
  else
    rear = rear - 1;
}

template <class T>
int Deque<T>::getFront()
{
  // check whether Deque is empty or not
  if (isEmpty()) {
    cout << " Underflow\n" << endl;
    return -1;
  }
  return arr[front];
}

template <class T>
int Deque<T>::getRear()
{
  // check whether Deque is empty or not
  if (isEmpty() || rear < 0) {
    cout << " Underflow\n" << endl;
    return -1;
  }
  return arr[rear];
}

#endif //GEOMETRY2_CIRCLE_DEQUE_H
