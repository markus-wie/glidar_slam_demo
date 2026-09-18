# YgorClustering

This is a C++ library for clustering data. It uses Boost.Geometry R*-trees for fast indexing. The DBSCAN implementation can cluster 20 million 2D datum in around an hour, and 20 thousand in seconds.

At the moment, only (vanilla) DBSCAN is implemented. It is based on the article "A Density-Based Algorithm for Discovering Clusters" by Ester, Kriegel, Sander, and Xu in 1996. DBSCAN is generally regarded as a solid, reliable clustering technique compared with techniques such as k-means (which is, for example, unable to cluster concave clusters).


## Origin

This library stems from Hal Clark, at the moment reachable under https://gitlab.com/hdeanclark/YgorClustering 

It implements the DBSCAN algorithm to cluster points.
