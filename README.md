# Shadtakovich
Get hands dirty by building a simple Rasterizer. 

## Miscellaneous rendering result
<img src="/Rasterizer_demo/Rasterizer_demo.001.png" width=50% height=50%>
<img src="/Rasterizer_demo/Rasterizer_demo.002.png" width=50% height=50%>
<img src="/Rasterizer_demo/Rasterizer_demo.003.png" width=50% height=50%>
<img src="/Rasterizer_demo/Rasterizer_demo.004.png" width=50% height=50%>


## Rendering Pipeline	
```mermaid
graph LR
A[3D Object e.g., mesh file] -->B[Read each face e.g., triangle represented primitive]
    B --> C[projection 3D -> 2D -> Screen]
    C -->D[draw fragment within the primitive]
    D -->E[depth test]
    E -->F[Fragment Shading]
    F -->G[Rendered Result]
```

## Current Support Functions
|  functions   | tested |
|  ----  | ----  |
| Bresenham's line drawing algorithm (https://stackoverflow.com/questions/10060046/drawing-lines-with-bresenhams-line-algorithm)  | <ul><li>- [x] </li> |
| Bresenham's Line drawing modified (https://github.com/ssloy/tinyrenderer/wiki/Lesson-1:-Bresenham%E2%80%99s-Line-Drawing-Algorithm) | <ul><li>- [x] </li> |
| Midpoint line drawing algorithm (https://www.geeksforgeeks.org/mid-point-line-generation-algorithm/) | <ul><li>- [x] </li> |
|  Shadow mapping  |  <ul><li>- [x] </li> |
|  Cascaded Shadow mapping  |   |		
| Anti-aliasing(MSAA) |  <ul><li>- [x] </li>|
| Anti-aliasing(FSAA) |  |
| Interaction (e.g. camera path) |  |	

## External Libraries	
Eigen.   
Opencv.   

	
	
	
	
## File Description
*  main.cpp
    > define transformation matrix (world->camera / camera->camera canonical / perspective + orthogonal transform / 3D canonical cube to 2D screen)  
    > define shader type (texture_fragment_shader / phong_fragment_shader)  
    > iosstream (read obj file / texture map; output rendered screen image)  
    > interaction (motion-based rendering from keyboard input)   
	
	
	

## Reference List:
https://github.com/ssloy/tinyrenderer	\
Games 101 Course (https://www.bilibili.com/video/BV1X7411F744?p=11)		\
Games 202 Course (https://www.bilibili.com/video/BV1YK4y1T7yY?p=10)		\
Shadertoy 		 (https://www.shadertoy.com/results?query=&sort=hot&filter=vr)	
