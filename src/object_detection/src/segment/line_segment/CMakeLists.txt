add_library(line_segment SHARED
            range_img_features.cpp  # using in tracking 
            RangeLineSegment.cpp 
            )

target_link_libraries(line_segment
                      ${OpenCV_LIBS}
                      ${PCL_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${THIRD_PARTY_LIBS}
                      )
