#ifndef LIDARIMAGE_H_
#define LIDARIMAGE_H_

#include <boost/iterator/iterator_facade.hpp>
#include <cstring>
#include <iostream>
#include <stdexcept>

/*!
 * \class LidarImage
 * \brief Provides a general template class for 2D image data
 *
 * The special feature is that is allows out-of-range access up to 1 pixel
 * Valid pixels are indexed in the range (0,0)..(horizSize-1,vertSize-1)
 * but access is possible up to (-1,-1)..(horizSize,vertSize)
 * WARNING: Access is not checked!!!!!
 *
 */

typedef std::pair<int, int> ColRowPair;

template <class stype>
class LidarImage {
 private:
  stype *data;
  int horizSize;  // size of valid data (without border)
  int vertSize;   // size of valid data (without border)

  //! calculates "memory address" for a given pixel coordinate
  inline int getDataIndex(const int h, const int v) const {
    return (v + 1) * (horizSize + 2) + h + 1;
  };
  inline void assertSameSize(const LidarImage &other);

  //////////////////////////////////////////////////////////////////
  //                    Inner class: iterator                     //
  //////////////////////////////////////////////////////////////////
  template <class Value>
  class iter
      : public boost::iterator_facade<iter<Value>, Value,
                                      boost::bidirectional_traversal_tag> {
   public:
    iter(Value *currPtr_, int currCol_, const int imgWidth_)
        : currPtr(currPtr_), currCol(currCol_), imgWidth(imgWidth_){};
    iter<Value> &operator=(const iter<Value> &other) {
      currPtr = other.currPtr;
      currCol = other.currCol;
      imgWidth = other.imgWidth;
      return *this;
    };
    Value &getLeftPixel() { return *(currPtr - 1); };
    Value &getRightPixel() { return *(currPtr + 1); };
    Value &getUpperPixel() { return *(currPtr + imgWidth + 2); };
    Value &getLowerPixel() { return *(currPtr - imgWidth - 2); };

   private:
    friend class boost::iterator_core_access;
    template <class>
    friend class iter;
    bool equal(iter<Value> const &other) const {
      return currPtr == other.currPtr;
    };
    iter<Value> &increment() {
      currPtr++;
      currCol++;
      if (currCol == imgWidth) {
        currPtr += 2;
        currCol = 0;
      };
      return *this;
    };
    iter<Value> &decrement() {
      currPtr--;
      currCol--;
      if (currCol == -1) {
        currPtr -= 2;
        currCol = imgWidth - 1;
      };
      return *this;
    };
    Value &dereference() const { return *currPtr; };

    Value *currPtr;
    int currCol;
    const int imgWidth;
  };  // end of iterator

 public:
  typedef LidarImage::iter<stype> iterator;
  typedef LidarImage::iter<stype const> const_iterator;
  // TODO (9): conversion from iterator to const_iterator doesn't work
  //  LidarImage<DVector>::iterator ptIt = frame->point3D.begin();
  //  LidarImage<DVector>::const_iterator ptEnd = frame->point3D.end();

  LidarImage(const unsigned int horizSize, const unsigned int vertSize);
  LidarImage(const LidarImage &other);
  LidarImage();
  virtual ~LidarImage();

  LidarImage &operator=(const LidarImage &other);
  void copyShallowFrom(const LidarImage &other);

  //!< returns the memory size in Bytes used by the object
  size_t getMemorySize() const;  
  inline stype &operator()(const int h, const int v) {
    return get(h, v);
  };  //!< allows up to 1 pixel out-of-range-access
  inline const stype &operator()(const int h, const int v) const {
    return get(h, v);
  };  //!< allows up to 1 pixel out-of-range-access
  inline const stype &get(
      const int h,
      const int v) const;  //!< allows up to 1 pixel out-of-range-access
  inline const stype &get(ColRowPair p) const {
    return get(p.first, p.second);
  };  //!< allows up to 1 pixel out-of-range-access
  inline stype &get(const int h,
                    const int v);  //!< allows up to 1 pixel out-of-range-access
  inline stype &get(ColRowPair p) {
    return get(p.first, p.second);
  };  //!< allows up to 1 pixel out-of-range-access
  inline stype *getAdr(const int h,
                       const int v);  //!< get memory address of pixel. Can be
                                      //!< misused to directly access lines
  inline stype *getAdr(ColRowPair p) {
    return get(p.first, p.second);
  };  //!< get memory address of pixel. Can be misused to directly access lines
  inline void set(
      const int h, const int v,
      const stype val);  //!< allows up to 1 pixel out-of-range-access
  inline void set(ColRowPair p, const stype val) {
    set(p.first, p.second, val);
  };  //!< allows up to 1 pixel out-of-range-access
  inline void fill(const stype val);  //!< fills all entries with this value,
                                      //!< also the 1 pixel out-of-range-area
  inline void getSize(int &h, int &v) const;
  inline int getHorizSize() const;
  inline int getVertSize() const;
  inline size_t size() const;

  inline iterator begin() {
    return iterator(&data[getDataIndex(0, 0)], 0, getHorizSize());
  };
  inline iterator end() {
    return iterator(&data[getDataIndex(0, getVertSize())], getHorizSize(),
                    getHorizSize());
  };
  inline const_iterator begin() const {
    return const_iterator(&data[getDataIndex(0, 0)], 0, getHorizSize());
  };
  inline const_iterator end() const {
    return const_iterator(&data[getDataIndex(0, getVertSize())], getHorizSize(),
                          getHorizSize());
  };

  inline ColRowPair firstValidCR(const stype &invalid) const {
    return nextValidCR(ColRowPair(-1, 0), invalid);
  };  //!< returns the first pixel that is not invalid
  
  inline ColRowPair nextValidCR(ColRowPair cr, const stype &invalid) const;  
  //!< returns the next pixel that is not invalid
  inline ColRowPair endCR() const { return ColRowPair(0, vertSize); };
};

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
////////////////////         Implementation           ////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
template <class stype>
LidarImage<stype>::LidarImage(const unsigned int horizSize_,
                              const unsigned int vertSize_)
    : horizSize((int)horizSize_), vertSize((int)vertSize_) {
  data = new stype[(horizSize + 2) * (vertSize + 2)];
  // one pixel boundary on each side
}

template <class stype>
LidarImage<stype>::LidarImage(const LidarImage &other)
    : horizSize(other.horizSize), vertSize(other.vertSize) {
  int size = (horizSize + 2) * (vertSize + 2);
  // one pixel boundary on each side
  data = new stype[size];
  memcpy(data, other.data, size * sizeof(stype));
  std::cout << " copied lidar image " << std::flush;
}

template <class stype>
LidarImage<stype>::LidarImage() : horizSize(0), vertSize(0) {
  data = new stype[1];
}

template <class stype>
LidarImage<stype>::~LidarImage() {
  delete[] data;
}

template <class stype>
void LidarImage<stype>::assertSameSize(const LidarImage<stype> &other) {
  if ((other.horizSize != horizSize) || (other.vertSize != vertSize)) {
    delete[] data;
    horizSize = other.horizSize;
    vertSize = other.vertSize;
    data = new stype[(horizSize + 2) * (vertSize + 2)];
    // throw std::range_error("LidarImage operator= : size differs!");
  }
}

template <class stype>
LidarImage<stype> &LidarImage<stype>::operator=(
    const LidarImage<stype> &other) {
  assertSameSize(other);
  // memcpy will not work in case stype hides a pointer => unintended shallow
  // copy
  stype *dTrg = data;
  stype *dSrc = other.data;
  int limit = (horizSize + 2) * (vertSize + 2);
  for (int i = 0; i < limit; ++i) {
    *dTrg = *dSrc;  
    // the only valid way: calling operator= on stype for each element
    ++dTrg;
    ++dSrc;
  }
  return *this;
}

template <class stype>
void LidarImage<stype>::copyShallowFrom(const LidarImage<stype> &other) {
  assertSameSize(other);
  memcpy(data, other.data, getMemorySize());
}

template <class stype>
inline size_t LidarImage<stype>::getMemorySize() const {
  return (horizSize + 2) * (vertSize + 2) * sizeof(stype);
}

template <class stype>
inline const stype &LidarImage<stype>::get(const int h, const int v) const {
  int idx = getDataIndex(h, v);
  return data[idx];
}

template <class stype>
inline stype &LidarImage<stype>::get(const int h, const int v) {
  int idx = getDataIndex(h, v);
  return data[idx];
}

template <class stype>
inline stype *LidarImage<stype>::getAdr(const int h, const int v) {
  int idx = getDataIndex(h, v);
  return &data[idx];
}

template <class stype>
inline void LidarImage<stype>::set(const int h, const int v, const stype val) {
  int idx = getDataIndex(h, v);
  data[idx] = val;
}

template <class stype>
inline void LidarImage<stype>::fill(const stype val) {
  int limit = (horizSize + 2) * (vertSize + 2);
  for (int idx = 0; idx < limit; ++idx) {
    data[idx] = val;
  }
}

template <class stype>
inline void LidarImage<stype>::getSize(int &h, int &v) const {
  h = horizSize;
  v = vertSize;
}

template <class stype>
inline int LidarImage<stype>::getHorizSize() const {
  return horizSize;
}

template <class stype>
inline int LidarImage<stype>::getVertSize() const {
  return vertSize;
}

template <class stype>
inline size_t LidarImage<stype>::size() const {
  return horizSize * vertSize;
}

template <class stype>
inline ColRowPair LidarImage<stype>::nextValidCR(ColRowPair cr,
                                                 const stype &invalid) const {
  int &col = cr.first;   // alias
  int &row = cr.second;  // alias
  do {
    ++col;
    if (col >= horizSize) {
      col = 0;
      ++row;
    }
  } while ((row < vertSize) && (col < horizSize) && (get(cr) == invalid));
  return cr;
}

#endif /*LIDARIMAGE_H_*/
