#ifndef RANGE_LINE_SEGMENT_H_
#define RANGE_LINE_SEGMENT_H_

#include "object_d/common_include.h"

// pair是将2个数据组合成一个数据，当需要这样的需求时就可以使用pair，
// 如stl中的map就是将key和value放在一起来保存。(python: dict)
typedef std::pair<int, int> ColRowPair;
/*!
 * \class RangeLineSegment
 * \brief A class that can be used to create a "segmentation forest"
 *
 * Each pixel of the image has a RangeLineSegment object. The initial state is
 * "valid" and "leaf/noProxy" meaning that each Pixel represents one segment.
 * When two pixels merge into one segment, one pixel will pass all its data
 * to the other "master pixel" and just store its pointer, turning into a
 * "proxy".
 * This can be done multiple times, thus a segmentation forest will
 * be created where a pixel references another pixel, which again references
 * a different one.
 * If a pixel is proxy, all function calls retrieving/modifying information
 * will be directed to the leaf.
 * A pixel can also be deactivated (turned "invalid"), meaning this pixel
 * has no valid segment assigned.
 */
class RangeLineSegment {
 public:
  //  typedef boost::shared_ptr< RangeLineSegment > SPtr;
  typedef std::list<ColRowPair>::const_iterator ColRowIterator;
  typedef std::list<ColRowPair>::const_iterator ColRowConstIterator;
  typedef std::map<RangeLineSegment *, float>::const_iterator NeighborIterator;

  RangeLineSegment();
  virtual ~RangeLineSegment();

  void reset();  //!< reset segement, activating it and removing the use as proxy
  void update();  //!< trace all pointers to other segments down to leafs,
                  //!< assure non-self-neighborness and unique occurance of
                  //! each neighbor in list
  void merge(RangeLineSegment *other);
  //!< merge two segments: moves all contents of other
  //!< segment into this, turning the other into a proxy

  RangeLineSegment *getPtr() {
    return linkTo ? linkTo->getPtr() : this;
  };
  //!< returns the pointer to the leaf in the segmentation forest

  const RangeLineSegment *getPtr() const {
    return linkTo ? linkTo->getPtr() : this;
  };  //!< returns the pointer to the leaf in the segmentation forest

  bool isProxy() const {
    return (bool)linkTo;
  };  //!< returns true if the current object serves as proxy, i.e. directs all
      //!< calls to another segment

  bool isValid() const {
    return getPtr()->valid;
  };  //!< returns true if the segment is valid, false otherwise

  void setValid(bool val);
  //!< can be used to mark the segment as valid/invalid

  float getColorR() const { return getPtr()->colorR; };
  float getColorG() const { return getPtr()->colorG; };
  float getColorB() const { return getPtr()->colorB; };
  void setColor(float r, float g, float b) {
    RangeLineSegment *s = getPtr();
    s->colorR = r;
    s->colorG = g;
    s->colorB = b;
  };

  void addPix(ColRowPair cr) {
    RangeLineSegment *s = getPtr();
    s->colRows.push_back(cr);
    s->pxCount++;
  };

  unsigned int getPxCount() const { return getPtr()->pxCount; };

  void getCenter(float &colC, float &rowC) const;
  void getColRows(ColRowIterator &begin, ColRowIterator &end,
                  unsigned int &count);
  void getColRows(ColRowConstIterator &begin, ColRowConstIterator &end,
                  unsigned int &count) const;

  static void makeNeighbors(RangeLineSegment *s1, RangeLineSegment *s2,
                            float maxSegScore);

  // connect 2 segments as neighbors with the specified segmentation score
  bool hasNeighbor(RangeLineSegment *s) {
    return (neighbors.find(s) != neighbors.end());
  };

  float getNeighborSegScore(RangeLineSegment *s);
  // returns the maximum segmentation score to the specified

  // segment or a value <0 if there is no connection
  void getNeighbors(NeighborIterator &begin, NeighborIterator &end,
                    unsigned int &count);

 private:
  RangeLineSegment *linkTo;  // !=NULL if it serves as proxy and redirects all
                             // queries to the Segment pointed to
  bool valid;
  float colorR;
  float colorG;
  float colorB;
  unsigned int pxCount;  // number of pixels belonging to this segment
  // std::list是双向链表，是一个允许在序列中任何一处位置以常量耗时插入或删除元素且
  // 可以双向迭代的顺序容器。std::list中的每个元素保存了定位前一个元素及后一个元素的信息，
  //允许在任何一处位置以常量耗时进行插入或删除操作，但不能进行直接随机访问。
  std::list<ColRowPair> colRows;
  // pixels of this segment. implicitly referencing the frame this
  // segment is stored in
  // 类似python中的dict
  std::map<RangeLineSegment *, float> neighbors;
  // pointers no neighboring segments in the image together with
  // maximum segmentation score of any link (which should be
  // below the segmentation threshold)
};

// struct SegmentSizeComp { bool operator()(const RangeLineSegment::SPtr &p1,
// const RangeLineSegment::SPtr &p2) const
//                         {return (p1->ptCount > p2->ptCount);}
//                       };
// struct SegmentVisibleSizeComp { bool operator()(const RangeLineSegment::SPtr
// &p1, const RangeLineSegment::SPtr &p2) const
//                         {return (p1->ptCountVisibleNextFrame >
//                         p2->ptCountVisibleNextFrame);}
//                       };

#endif /*RANGE_LINE_SEGMENT_H_*/
