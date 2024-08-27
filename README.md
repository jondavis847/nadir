WGPU Notes:

-We need to be able to store data on the GPU
-We do that with buffers
-A Vertex buffer will store the data that is common for all types of that shape
-An instance buffer stores the data for each instance
    - for example, vertex buffers might have the given vertex array layout and generalized values for a given shape
    - instance data would then have appropriate scale factors and location and rotation of the vertices for a given object