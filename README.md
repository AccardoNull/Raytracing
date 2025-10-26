# Raytracing
A raytracer that can generate objects then apply different effects, sample input files included.
Features:
- Perspective camera with user-defined image plane (LEFT, RIGHT, BOTTOM, TOP, NEAR)
- Resolution control via RES keyword
- Full scene parsing from input file (supports SPHERE, LIGHT, AMBIENT, BACKGROUND, OUTPUT)
- Multiple light sources with color and position
- Ray-sphere intersection with support for arbitrary ellipsoid scaling
- Use of homogeneous transformation matrices (M) and their inverses (M⁻¹)
- Ray transformation from world space to object space using M⁻¹ for intersection
- Normal transformation from object space to world space using M⁻¹ (ignores translation)
- Normal flipping when the ray starts inside the sphere
- Ambient, diffuse, and Phong specular lighting (per-light source)
- Shadow computation (includes soft skip when light ray originates inside a sphere)
- Reflections with recursive tracing up to a configurable depth (depth <= 3)
- Output image saved in P3 (ASCII) PPM format
- Near-plane clipping to approximate hollow objects or prevent artifacts
- Custom scaling factors (kr patching) to reduce unrealistic reflection intensity
- Efficient matrix inversion via adjoint and determinant functions
- Prevents self-shadowing artifacts using ray origin offset (epsilon = 1e-4)

Usage:
In Command Promot, navigate to the root file.

Compile: cl raytrace.cpp /FeRayTracer.exe

Run:     RayTracer.exe input.txt        //Sample input file provided in folder, e.g. testAmbient.txt

Output: e.g. testAmbient.ppm

Dependencies: None (standard C++ STL only)
