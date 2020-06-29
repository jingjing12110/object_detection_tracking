#include "RangeLineSegment.h"

#include <boost/foreach.hpp>
#include <boost/typeof/std/utility.hpp>  //BOOST_AUTO

RangeLineSegment::RangeLineSegment()
    : colorR(0.2 + 0.8 * (float)rand() / (float)RAND_MAX),
      colorG(0.2 + 0.8 * (float)rand() / (float)RAND_MAX),
      colorB(0.2 + 0.8 * (float)rand() / (float)RAND_MAX) {
  reset();
}

RangeLineSegment::~RangeLineSegment() {
  //  cerr << "~S";
}

void RangeLineSegment::reset() {
  linkTo = NULL;
  valid = false;
  pxCount = 0;
  colRows.clear();  // std::list
  neighbors.clear();  // std::map
}

void RangeLineSegment::update() {
  if (linkTo == NULL)
    return;  // this segment serves as leaf, so nothing has to be changed

  // trace pointer down to leaf for further speed improvements
  RangeLineSegment *newPtr = linkTo->getPtr();
  if (newPtr == linkTo)
    return;  // no changes will be made to pointer, so nothing else has to be
             // changed

  // change link in all neighbors
  for (BOOST_AUTO(nIt, neighbors.begin()); nIt != neighbors.end(); ++nIt) {
    RangeLineSegment *n = nIt->first;
    BOOST_AUTO(nnIt, n->neighbors.find(this));
    if (nnIt == n->neighbors.end()) {
      std::cerr << std::endl
          << "RangeLineSegment::update(): link to neighbor is not symmetric!!";
    } else {
      // check if leaf-connection already exists or not
      BOOST_AUTO(nlIt, neighbors.find(newPtr));
      if (nlIt == neighbors.end())  // no => create new link to leaf
        n->neighbors.insert(
            std::pair<RangeLineSegment *, float>(newPtr, nIt->second));
      else  // yes => modify segmentation score
        nlIt->second = fmax(nlIt->second, nIt->second);
        // store maximum connection score
      // erase current link
      n->neighbors.erase(nnIt);
    }
  }
  linkTo = newPtr;
}

// TODO (9): maybe it is faster to calculate neighbors/links etc after
// segmentation was finished?

void RangeLineSegment::merge(RangeLineSegment *otherSeg) {
  RangeLineSegment *currSeg = getPtr();  // this segment might by a proxy
                                         // itself, so first get target pointer
  otherSeg = otherSeg->getPtr();  // other segment might by a proxy too, so
                                  // first get target pointer
  if (currSeg == otherSeg)
    return;  // if both segments are the same, nothing has to be done. return
  if (!currSeg->valid) return;   // invalid segments cannot be merged
  if (!otherSeg->valid) return;  // invalid segments cannot be merged
  if (currSeg->pxCount < otherSeg->pxCount) {  
    // copy colors if other segment is larger
    currSeg->colorR = otherSeg->colorR;
    currSeg->colorG = otherSeg->colorG;
    currSeg->colorB = otherSeg->colorB;
  }
  // move all pixels into "this"
  currSeg->colRows.splice(currSeg->colRows.begin(), otherSeg->colRows);
  currSeg->pxCount += otherSeg->pxCount;
  otherSeg->pxCount = 0;
  // move all neighbors into "this", removing reciprocal link and correcting
  // targets link
  currSeg->neighbors.erase(otherSeg);
  for (BOOST_AUTO(otherSegNeighbIt, otherSeg->neighbors.begin());
       otherSegNeighbIt != otherSeg->neighbors.end(); ++otherSegNeighbIt) {
    RangeLineSegment *otherSegNeighb = otherSegNeighbIt->first;
    if (otherSegNeighb == currSeg)
      continue;  // will be erased by clearing below
    BOOST_AUTO(currSegNeighbIt, currSeg->neighbors.find(otherSegNeighb));
    if (currSegNeighbIt ==currSeg->neighbors.end()) {
      // neighbor is not yet linked by current segment, so:
      currSeg->neighbors.insert(*otherSegNeighbIt);  
      // just add bidirectional
      otherSegNeighb->neighbors.insert(
          std::pair<RangeLineSegment *, float>(currSeg, otherSegNeighbIt->second));
    } else {  // neighbor is already linked, so:
      if (otherSegNeighbIt->second > currSegNeighbIt->second) {
        currSegNeighbIt->second = otherSegNeighbIt->second;  
        // update segmentation score
        BOOST_AUTO(tmpit, otherSegNeighb->neighbors.find(currSeg));  
        // correct target's link
        tmpit->second = otherSegNeighbIt->second;
      }
    }
    otherSegNeighb->neighbors.erase(otherSeg); 
    // delete link from neighbor to other segment
  }
  otherSeg->neighbors.clear();
  otherSeg->linkTo = currSeg;
}

void RangeLineSegment::setValid(bool val) {
  RangeLineSegment *s = getPtr();  // this segment might by a proxy itself, so
                                   // first get target pointer
  s->valid = val;
  if (!val) {
    // remove link to this segment from all neighbors
    for (BOOST_AUTO(ni, s->neighbors.begin()); ni != s->neighbors.end(); ++ni) {
      ni->first->neighbors.erase(s);
    }
    s->neighbors.clear();
  }
}

void RangeLineSegment::getCenter(float &colC, float &rowC) const {
  const RangeLineSegment *s = getPtr();  // this segment might by a proxy
                                         // itself, so first get target pointer
  colC = 0.0;
  rowC = 0.0;
  BOOST_FOREACH (const ColRowPair &cr, s->colRows) {
    colC += cr.first;
    rowC += cr.second;
  }
  colC /= (float)s->pxCount;
  rowC /= (float)s->pxCount;
}

void RangeLineSegment::getColRows(RangeLineSegment::ColRowIterator &begin,
                                  RangeLineSegment::ColRowIterator &end,
                                  unsigned int &count) {
  RangeLineSegment *s = getPtr();  // this segment might by a proxy itself, so
                                   // first get target pointer
  begin = s->colRows.begin();
  end = s->colRows.end();
  count = s->pxCount;
}

void RangeLineSegment::getColRows(RangeLineSegment::ColRowConstIterator &begin,
                                  RangeLineSegment::ColRowConstIterator &end,
                                  unsigned int &count) const {
  const RangeLineSegment *s = getPtr();  // this segment might by a proxy
                                         // itself, so first get target pointer
  begin = s->colRows.begin();
  end = s->colRows.end();
  count = s->pxCount;
}

void RangeLineSegment::makeNeighbors(RangeLineSegment *s1, RangeLineSegment *s2,
                                     float maxSegScore) {
  s1 = s1->getPtr();
  s2 = s2->getPtr();
  if (s1 == s2)
    return;  // if both segments are the same, nothing has to be done. return
  BOOST_AUTO(nit1, s1->neighbors.find(s2));
  if (nit1 == s1->neighbors.end()) {
    // not yet neighbors, so create 2 links
    s1->neighbors.insert(std::pair<RangeLineSegment *, float>(s2, maxSegScore));
    s2->neighbors.insert(std::pair<RangeLineSegment *, float>(s1, maxSegScore));
  } else {
    // neighbor-link already exists, so possibly overwrite current score
    if (maxSegScore > nit1->second) {
      nit1->second = maxSegScore;
      BOOST_AUTO(nit2, s2->neighbors.find(s1));
      // links are symmetric, so this operation must succeed!
      nit2->second = maxSegScore;
    }
  }
}

float RangeLineSegment::getNeighborSegScore(RangeLineSegment *n) {
  RangeLineSegment *s = getPtr();
  BOOST_AUTO(nit, s->neighbors.find(n));
  if (nit == s->neighbors.end())
    return -1.0f;
  else
    return nit->second;
}

void RangeLineSegment::getNeighbors(RangeLineSegment::NeighborIterator &begin,
                                    RangeLineSegment::NeighborIterator &end,
                                    unsigned int &count) {
  RangeLineSegment *s = getPtr();  // this segment might by a proxy itself, so
                                   // first get target pointer
  begin = s->neighbors.begin();
  end = s->neighbors.end();
  count = s->neighbors.size();
}
