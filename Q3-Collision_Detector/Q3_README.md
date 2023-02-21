# Q3-Collision_Detector
Again, I can't use rviz because I was using docker which has x service related errors. So I make it as a pure python problem. And I only implement the 3D cases. 

- What is the running time complexity of your algorithm?\
The first two functions (`point_in_box`, `box_intersect`) are pretty simple and both of them are $O(1)$. The interesting part is `optimized_function` which implements the third function. I used a spatial indexing structure to store all points of all boxes. Usually, in practically, we can use kdtree and Octree to implement the structure. But here I don't want to make them to complicated. I just use a simple sorted array to represent the structure. Here is the basic idea:
1. collect all points including all 8 pints in all boxes and all query points (Assuming we have m boxes and n query points)
2. sort the list using the distance of the points as the key. $O((m+n)log(m+n))$
3. for each box, each possible points in the box must be within the distance of `min_point` and `max_point` which are the main diagonal of the box. We can use $O(log(m + n))$ to find the two points in the above list. And iterate all the points in the middle to check if they are in the box.\
The whole time complexity would be $(m+n)log(m+n)$
- What is the memory complexity of your algorithm?\
$O(m+n)$
- Describe how your algorithm could be parallelized with a GPU (no coding needed)\
Actually, each iterations are completely independent, so we can iterate them separately. In the memory part, we can put the sorted list into the shared memory so that each sm can access them freely. And after the memory is loaded successfully, we can access the list without any mutex because we only need to read the data instead of changing them.