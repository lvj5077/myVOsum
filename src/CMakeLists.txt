project(myVOsum)

ADD_EXECUTABLE( main main.cpp) # always test cmake function first

INCLUDE_DIRECTORIES ("/usr/local/include/eigen3/")
INCLUDE_DIRECTORIES ("/usr/local/include/eigen3/")

# # SET("PCL_DIR" "/home/jin/Third_party_packages/pcl-pcl-1.9.1/build")
# SET(PCL_DIR "/usr/local/Cellar/pcl/")
SET(PCL_DIR "/usr/local/Cellar/pcl/1.9.1_4/share/pcl-1.9")
FIND_PACKAGE( PCL 1.9 REQUIRED COMPONENTS common io visualization filters )
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )
link_directories( /usr/local/Cellar/flann/1.9.1_7/lib )

# SET(OpenCV_DIR "/Users/lingqiujin/Third_Party_Packages/opencv-2.4.13.6/build")
# find_package( OpenCV 2.4 REQUIRED )
# SET(OpenCV_DIR "/Users/lingqiujin/Third_Party_Packages/opencv-3.4.5/build")
SET(OpenCV_DIR "/usr/local/Cellar/opencv@3/3.4.5_1/share/OpenCV")

find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
link_directories(${OpenCV_LIB_DIR})

# include_directories( /usr/local/Cellar/opencv@3/3.4.5_1/include )
# set(SIFT_LIBS "/usr/local/Cellar/opencv@3/3.4.5_1/lib/libopencv_xfeatures2d.dylib" )

# include_directories( /usr/local/Cellar/flann/1.9.1_7/include )
# link_directories( /usr/local/Cellar/flann/1.9.1_7/lib )

SET(G2O_DIR "/Users/lingqiujin/Third_Party_Packages/g2o")
SET(G2O_LIB "/Users/lingqiujin/Third_Party_Packages/g2o/lib/")
include_directories( ${G2O_DIR} )
link_directories( ${G2O_LIB} )

FIND_PACKAGE( G2O REQUIRED )

SET(CSparse_include_DIR "/usr/local/Cellar/suite-sparse/5.3.0/include/")
SET(CSparse_include_LIB "/usr/local/Cellar/suite-sparse/5.3.0/lib/")
include_directories( ${CSparse_include_DIR} )
link_directories( ${CSparse_include_LIB} )



ADD_LIBRARY( slamBase slamBase.cpp )
TARGET_LINK_LIBRARIES( slamBase
   ${OpenCV_LIBS}
   # ${SIFT_LIBS}
   ${PCL_LIBRARIES}
   # /usr/local/Cellar/flann/1.9.1_7/lib/libflann.dylib
   # /usr/local/Cellar/flann/1.9.1_7/lib
)

ADD_LIBRARY( myG2Oedge myG2Oedge.cpp )
TARGET_LINK_LIBRARIES( myG2Oedge
   g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension g2o_solver_csparse g2o_types_slam3d
   ${CSparse_include_LIB} 
)


ADD_LIBRARY( pose_estimation pose_estimation.cpp )
TARGET_LINK_LIBRARIES( pose_estimation
   ${OpenCV_LIBS}
   myG2Oedge
)

ADD_EXECUTABLE( testT testT.cpp )
TARGET_LINK_LIBRARIES( testT
	slamBase
	pose_estimation
	${OpenCV_LIBS}
	${PCL_LIBRARIES}
)

# ADD_EXECUTABLE( mySLAM mySLAM.cpp )
# TARGET_LINK_LIBRARIES( mySLAM
#    slamBase
#    pose_estimation
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
# )

# add_executable( feature_extraction feature_extraction.cpp  )
# target_link_libraries( feature_extraction ${OpenCV_LIBS} )

# # add_executable( pose_estimation_2d2d pose_estimation_2d2d.cpp extra.cpp ) # use this if in OpenCV2 
# add_executable( pose_estimation_2d2d pose_estimation_2d2d.cpp )
# target_link_libraries( pose_estimation_2d2d ${OpenCV_LIBS} )

# # add_executable( triangulation triangulation.cpp extra.cpp) # use this if in opencv2 
# add_executable( triangulation triangulation.cpp )
# target_link_libraries( triangulation ${OpenCV_LIBS} )

# add_executable( pose_estimation_3d2d pose_estimation_3d2d.cpp )
# target_link_libraries( pose_estimation_3d2d 
#    ${OpenCV_LIBS}
#    ${CSparse_include_LIB} 
#    g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
# )

# add_executable( pose_estimation_3d3d pose_estimation_3d3d.cpp )
# target_link_libraries( pose_estimation_3d3d 
#    ${OpenCV_LIBS}
#    g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension 
#    ${CSparse_include_LIB} 
# )
