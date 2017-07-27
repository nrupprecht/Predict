// To be included in aligned_array.hpp

template<typename T> aligned_array<T>::aligned_array() : _size(0), _alignment(64), data(0) {};

template<typename T> aligned_array<T>::aligned_array(int s) : _size(s), _alignment(64) {
  //data = alignedAlloc(_alignment, _size);
  alignedAlloc(data, _alignment, _size);
  // Initialize the entries to default values
  for (int i=0; i<s; ++i) data[i] = T();
}

template<typename T> aligned_array<T>::aligned_array(int s, const T& d) : _size(s), _alignment(64) {
  //data = alignedAlloc(_alignment, _size);
  alignedAlloc(data, _alignment, _size);
  int i;
  // Set the first s entries to the requested value
  for (i=0; i<s; ++i)   data[i] = d;
}

template<typename T> aligned_array<T>::~aligned_array() {
  if (data) free(data);
  data = 0;
}

template<typename T> void aligned_array<T>::reserve(int ns) {
  //T* new_data = alignedAlloc(_alignment, ns);
  T *new_data;
  alignedAlloc(new_data, _alignment, ns);
  if (data) {
    int end = _size<ns ? _size : ns;
    int i;
    for (i=0; i<end; ++i) new_data[i] = data[i];
    for (; i<ns; ++i)     new_data[i] = T();
    // Free old data
    free(data);
  }
  else 
    for (int i=0; i<ns; ++i) new_data[i] = T();
  // Set new array and size
  data = new_data;
  _size = ns;
}

template<typename T> void aligned_array<T>::reserve(int ns, const T& value) {
  //T* new_data = alignedAlloc(_alignment, ns);
  T *new_data;
  alignedAlloc(new_data, _alignment, ns);
  if (data) {
    int end = _size<ns ? _size : ns;
    int i;
    for (i=0; i<end; ++i) new_data[i] = data[i];
    for (; i<ns; ++i)     new_data[i] = value;
    // Free old data
    free(data);
  }
  else
    for (int i=0; i<ns; ++i) new_data[i] = T();
  // Set new array and size
  data = new_data;
  _size = ns;
}

template<typename T> void aligned_array<T>::reserve2(int oldFirst, int newFirst, int oldSecond, int newSecond) {
  unsigned int ns = newFirst + newSecond;
  //T* new_data = alignedAlloc(_alignment, ns);
  T *new_data;
  alignedAlloc(new_data, _alignment, ns);
  if (data) {
    int i;
    int index1 = min(min(oldFirst,  newFirst),  _size);
    int index2 = min(min(oldSecond, newSecond), _size-oldFirst);
    for (i=0; i<index1; ++i) new_data[i] = data[i];
    for (; i<newFirst; ++i)  new_data[i] = T();
    for (i=0; i<index2; ++i) new_data[newFirst+i] = data[oldFirst+i];
    for (; i<newSecond; ++i) new_data[i] = T();
    
    // Free old data
    free(data);
  }
  data = new_data;
  _size = ns;
}

template<typename T> void aligned_array<T>::reserve2(int oldFirst, int newFirst, int oldSecond, int newSecond, const T& value) {
  int ns = newFirst + newSecond;
  // T* new_data = alignedAlloc(_alignment, ns);
  T *new_data;
  alignedAlloc(new_data, _alignment, ns);
  if (data) {
    int i;
    int index1 = min(min(oldFirst,  newFirst),  _size);
    int index2 = min(min(oldSecond, newSecond), _size-oldFirst);
    for (i=0; i<index1; ++i) new_data[i] = data[i];
    for (; i<newFirst; ++i)  new_data[i] = value;
    for (i=0; i<index2; ++i) new_data[newFirst+i] = data[oldFirst+i];
    for (; i<newSecond; ++i) new_data[i] = value;

    // Free old data
    free(data);
  }
  data = new_data;
  _size = ns;
}

template<typename T> void aligned_array<T>::resize(int s) {
  if (data) free(data);
  _size = s;
  alignedAlloc(data, _alignment, _size);
  // Set values
  for (int i=0; i<s; ++i) data[i] = T();
}

template<typename T> void aligned_array<T>::resize(int s, T v) {
  if (data) free(data);
  _size = s;
  alignedAlloc(data, _alignment, _size);
  // Set values
  for (int i=0; i<s; ++i) data[i] = v;
}

template<typename T> T& aligned_array<T>::at(int i) {
  if (i<0 || _size<=i) throw aligned_array_out_of_bounds(i);
  return data[i];
}

template<typename T> T aligned_array<T>::at(int i) const {
  if (i<0 || _size<=i) throw aligned_array_out_of_bounds(i);
  return data[i];
}

template<typename T> T& aligned_array<T>::operator[] (int i) {
  return data[i];
}

template<typename T> T aligned_array<T>::operator[] (int i) const {
  return data[i];
}

template<typename T> void aligned_array<T>::setAlignment(int a) {
  if (a<0) throw bad_alignment(a);
  _alignment = a;
  reserve(_size);
}

template<typename T> void aligned_array<T>::clearValues() {
  unsigned int num = _size*sizeof(T);
  memset(data, 0, num);
}

template<typename T> bool aligned_array<T>::iterator::operator==(aligned_array<T>::iterator p) {
  return (_point==p._point && _size==p._size && data==p.data);
}

template<typename T> bool aligned_array<T>::iterator::operator!=(aligned_array<T>::iterator p) {
  return (_point!=p._point || _size!=p._size || data!=p.data);
}

template<typename T> aligned_array<T>::iterator& aligned_array<T>::iterator::operator++() {
  ++_point;
  return *this;
}

template<typename T> T& aligned_array<T>::iterator::operator*() {
  return data[_point];
}

template<typename T> aligned_array<T>::iterator::iterator(T* d, int s) : _point(0), _size(s), data(d) {};

template<typename T> aligned_array<T>::iterator::iterator(T* d, int s, int p) : _point(p), _size(s), data(d) {}
