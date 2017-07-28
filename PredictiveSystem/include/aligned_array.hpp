/*
 * Author: Nathaniel Rupprecht
 * Start Data: May 12, 2017
 *
 */

#ifndef __ALIGNED_ARRAY_HPP__
#define __ALIGNED_ARRAY_HPP__

#include <cstdlib>
#include <cstdio>
#include <stdlib.h>

#include "Utility.hpp"

// For definition of _POSIX_
#include "Settings.hpp"

namespace Predictive {

  /*
   * Define if this way mainly because my mac doesn't do aligned_alloc, but can do posix_memalign
   */
  template<typename T> inline void alignedAlloc(T *& pointer, size_t alignment, size_t size) {
#if _POSIX_MEMALIGN_ == 1
    posix_memalign((void**)(&pointer), static_cast<size_t>(alignment), static_cast<size_t>(size*sizeof(T)));
#else
    pointer = (T*) aligned_alloc(alignment, size*sizeof(T));
#endif
  }
  
  /*
   * @class aligned_array
   * Stores aligned data, like a vector
   */
  template<typename T> class aligned_array {
  public:
    // Default constructor
    aligned_array();
    
    // Presized array constructor
    aligned_array(int);
    
    // Fill array constructor
    aligned_array(int, const T&);
    
    // Destructor
    ~aligned_array();

    // Resize the array
    void reserve(int);
    // Set unused spots to have a specific value
    void reserve(int, const T&);

    // Resize the array in two parts
    void reserve2(int, int, int, int);
    // Set unused spots to have a specific value
    void reserve2(int, int, int, int, const T&); 

    // Resize and clear values
    void resize(int);
    // Resize and set values
    void resize(int, T);
    // Reset
    void reset();

    // Data access
    T& at(int);
    T  at(int) const;
    T& operator[] (int);
    T  operator[] (int) const;
    
    // Get pointer
    T* getPtr() { return data; }

    // Accessors
    int size()   const { return _size; }
    bool empty() const { return _size==0; }

    // *** Mutators
    void setAlignment(int);

    // Set all values to zero
    void clearValues();

    // Iterator class
    class iterator {
    public:
      // Comparison
      bool operator==(iterator);
      bool operator!=(iterator);
      // Incrementation (prefix)
      iterator& operator++();
      // Dereferencing operator
      T& operator*();
      
    private:
      // Private constructors
      iterator(T*, int);
      iterator(T*, int, int);
      // Data
      int _point;
      int _size;
      T* data;

      // Make the owner class a friend class so it can access private constructors
      friend class aligned_array;
    };

    // Iterator related functions
    iterator begin() {
      return iterator(data, _size);
    }
    
    iterator end() {
      return iterator(data, _size, _size);
    }

    // Exception classes: Out of bounds
    class aligned_array_out_of_bounds {
    public:
      aligned_array_out_of_bounds(int i) : index(i) {};
      int index;
    };

    // Exception classes: bad choice of alignment
    class bad_alignment {
    public:
      bad_alignment(int a) : align(a) {};
      int align;
    };
      
  private:
    int _size;
    unsigned int _alignment;
    
    T* data;
  };

#include "aligned_array.cpp"
  
}
#endif // __ALIGNED_ARRAY_HPP__
