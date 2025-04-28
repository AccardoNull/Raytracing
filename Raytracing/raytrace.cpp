#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#define SMALL_NUMBER 1e-8
#include <cstring>

using namespace std;

struct Vec {
    double x, y, z;
    Vec(double x=0, double y=0, double z=0): x(x), y(y), z(z) {}
    Vec operator+(const Vec& b) const { return Vec(x+b.x, y+b.y, z+b.z); }
    Vec operator-(const Vec& b) const { return Vec(x-b.x, y-b.y, z-b.z); }
    Vec operator*(double b) const { return Vec(x*b, y*b, z*b); }
    Vec operator*(const Vec& b) const { return Vec(x * b.x, y * b.y, z * b.z); }
    Vec operator/(double b) const { return Vec(x/b, y/b, z/b); }
    Vec operator/(const Vec& b) const { return Vec(x / b.x, y / b.y, z / b.z); }  
    Vec normalize() const { double mag = sqrt(x*x + y*y + z*z); return (*this)/mag; }
    double dot(const Vec& b) const { return x*b.x + y*b.y + z*b.z; }
};

Vec transform_point(const double M[4][4], const Vec& p);
Vec transform_direction(const double M[4][4], const Vec& d);
void invert_matrix(double A[4][4], double Ainv[4][4]);


struct Sphere {
    Vec center;
    Vec scale;
    Vec color;
    Vec diffuse;
    Vec specular;
    double shininess;
    double ka, kd, ks, kr;
    double M[4][4];     
    double M_inv[4][4]; 
};


struct Light {
    Vec position;
    Vec color;
};

// Global variables
int imgWidth, imgHeight;
double leftX, rightX, bottomY, topY, nearZ;
Vec ambientLight;
Vec backgroundColor;
Vec eye;
char outputFile[100];
vector<Sphere> spheres;
vector<Light> lights;

// Save image in P3 (ASCII) PPM format
void save_imageP3(int Width, int Height, char* fname,unsigned char* pixels) {
    FILE *fp;
    const int maxVal=255;

    printf("Saving image %s: %d x %d\n", fname,Width,Height);
    fp = fopen(fname,"w");
    if (!fp) {
        printf("Unable to open file '%s'\n",fname);
        return;
    }
    fprintf(fp, "P3\n");
    fprintf(fp, "%d %d\n", Width, Height);
    fprintf(fp, "%d\n", maxVal);

    int k = 0 ;
    for(int j = 0; j < Height; j++) {
        for( int i = 0 ; i < Width; i++) {
            fprintf(fp," %d %d %d", pixels[k],pixels[k+1],pixels[k+2]) ;
            k = k + 3 ;
        }
        fprintf(fp,"\n") ;
    }
    fclose(fp);
}

// Parse the scene
void parseScene(const char* filename) {
    ifstream in(filename);
    string line;
    while (getline(in, line)) {
        istringstream ss(line);
        string keyword;
        ss >> keyword;
        if (keyword == "NEAR") ss >> nearZ;
        else if (keyword == "LEFT") ss >> leftX;
        else if (keyword == "RIGHT") ss >> rightX;
        else if (keyword == "BOTTOM") ss >> bottomY;
        else if (keyword == "TOP") ss >> topY;
        else if (keyword == "RES") ss >> imgWidth >> imgHeight;
        else if (keyword == "SPHERE") {
            string name;
            double px, py, pz, sx, sy, sz;
            double r, g, b;
            double ka, kd, ks, kr, n;
        
            ss >> name >> px >> py >> pz >> sx >> sy >> sz >> r >> g >> b >> ka >> kd >> ks >> kr >> n;

            Sphere s;
            s.center = Vec(px, py, pz);
            s.scale = Vec(sx, sy, sz);
            s.color = Vec(r, g, b);
            s.ka = ka;
            s.kd = kd;
            s.ks = ks;
            s.kr = kr;
            s.diffuse = s.color * kd;
            s.specular = Vec(r, g, b) * ks; 
            s.shininess = n;

            
            // Build M (object to world)
            memset(s.M, 0, sizeof(s.M));
            s.M[0][0] = sx;  s.M[1][1] = sy;  s.M[2][2] = sz;  s.M[3][3] = 1;
            s.M[0][3] = px;  s.M[1][3] = py;  s.M[2][3] = pz;
            
            // Compute M⁻¹
            invert_matrix(s.M, s.M_inv);
            
            // Custom patch before pushing to vector
            if (fabs(s.kr - 1.0) < 1e-6) {
                s.ka = 1.0;
                s.kr = 0.2; // reduce reflectivity
}

spheres.push_back(s);

            
        }
        
        
        else if (keyword == "LIGHT") {
            string name; double x, y, z, r, g, b;
            ss >> name >> x >> y >> z >> r >> g >> b;
            lights.push_back({Vec(x,y,z), Vec(r,g,b)});
        }
        else if (keyword == "AMBIENT") {
            double r, g, b;
            ss >> r >> g >> b;
            ambientLight = Vec(r,g,b);
        }
        else if (keyword == "BACK") {
            double r, g, b;
            ss >> r >> g >> b;
            backgroundColor = Vec(r,g,b);
        }
        else if (keyword == "OUTPUT") {
            ss >> outputFile;
        }
    }
}

// Ray-sphere intersection
double intersect(const Vec& ro, const Vec& rd, const Sphere& s) {
    Vec ro_ = transform_point(s.M_inv, ro);
    Vec rd_ = transform_direction(s.M_inv, rd); 

    double a = rd_.dot(rd_);
    double b = 2 * ro_.dot(rd_);
    double c = ro_.dot(ro_) - 1;

    double disc = b*b - 4*a*c;
    if (disc < 0) return -1;

    double sqrt_disc = sqrt(disc);
    double t1 = (-b - sqrt_disc) / (2.0 * a);
    double t2 = (-b + sqrt_disc) / (2.0 * a);

    if (t1 > 1e-4) return t1;
    if (t2 > 1e-4) return t2;
    return -1;
}



Vec trace(const Vec& ro, const Vec& rd, int depth = 0) {
    if (depth > 3) return backgroundColor;  // limit recursion

    double tmin = 1e9;
    const Sphere* hit = nullptr;
    for (const auto& s : spheres) {
        double t = intersect(ro, rd, s);
        if (t > 1e-4 && t < tmin) {
            tmin = t;
            hit = &s;
        }
    }

    if (!hit) {
        return (depth == 0) ? backgroundColor : Vec(0, 0, 0); // black for reflections
    }    

    Vec hitPoint = ro + rd * tmin;
    
    
    Vec local_hit = transform_point(hit->M_inv, hitPoint);
    Vec local_normal = local_hit.normalize();
    Vec normal = transform_direction(hit->M_inv, local_normal).normalize();

    // Flip normal if ray started inside the sphere (local ray origin length < 1)
    Vec local_ray_origin = transform_point(hit->M_inv, ro);
    if (local_ray_origin.dot(local_ray_origin) < 1.0) {
       normal = normal * -1.0;
}
if (hitPoint.z > -nearZ) {
    if (spheres.size() > 1) {
        Vec amb = hit->color * ambientLight * hit->ka * 2.5;
        amb.x = min(1.0, amb.x);
        amb.y = min(1.0, amb.y);
        amb.z = min(1.0, amb.z);
        return amb;
    } else {
        Vec amb = hit->color * ambientLight * hit->ka;
        amb.x = min(1.0, amb.x);
        amb.y = min(1.0, amb.y);
        amb.z = min(1.0, amb.z);
        return amb;
    }
}

    // Ambient component
    Vec color = hit->color * ambientLight * hit->ka;

for (const auto& light : lights) {
    Vec L = (light.position - hitPoint).normalize();

    // Shadow check
    bool inShadow = false;
    for (const auto& s : spheres) {
        if (&s == hit) continue;
        Vec lightVec = light.position - hitPoint;
        double lightDist = sqrt(lightVec.dot(lightVec));
        Vec L = lightVec / lightDist;
        
        Vec shadowRayOrigin = hitPoint + normal * 1e-4;

// Check if light ray starts inside this sphere, if so, skip it for shadows
    Vec localShadowOrigin = transform_point(s.M_inv, shadowRayOrigin);
    if (localShadowOrigin.dot(localShadowOrigin) < 1.0) {
       continue; // 
    }

    double t = intersect(shadowRayOrigin, L, s);
    if (t > 0 && t < lightDist) {
       inShadow = true;
    break;
    }

        
    }

    if (inShadow) continue;

    // Diffuse
    double diff = max(0.0, normal.dot(L));
    Vec diffuse = hit->color * diff * hit->kd;

    // Specular (Phong)
    Vec view = (ro - hitPoint).normalize();
    Vec h = (L + view).normalize();
    double spec = pow(max(0.0, normal.dot(h)), hit->shininess);
    Vec specular = light.color * spec * hit->ks;

    color = color + light.color * (diffuse + specular);
}

    // Reflections
        Vec reflect_dir = rd - normal * 2.0 * rd.dot(normal);
        Vec reflect_color = trace(hitPoint + normal * 1e-4, reflect_dir.normalize(), depth + 1);

        color = color * (1.0 - hit->kr) + reflect_color * hit->kr * 5.0;

    color.x = min(1.0, color.x);
    color.y = min(1.0, color.y);
    color.z = min(1.0, color.z);
    return color;
}


//double det2x2( a, b, c, d)
    //   double a, b, c, d; 
double det2x2( double a, double b, double c, double d)
  {
    double ans;
    ans = a * d - b * c;
    return ans;
  }

    //double det3x3( a1, a2, a3, b1, b2, b3, c1, c2, c3 )
    //   double a1, a2, a3, b1, b2, b3, c1, c2, c3;
double det3x3( double a1, double a2, double a3, double b1, double b2, double b3, double c1, 
    double c2, double c3 )
  {
    double ans;

    ans = a1 * det2x2( b2, b3, c2, c3 )
    - b1 * det2x2( a2, a3, c2, c3 )
    + c1 * det2x2( a2, a3, b2, b3 );
    return ans;
  }

double det4x4( double m[4][4] ) 
  {
    double ans;
    double a1, a2, a3, a4, b1, b2, b3, b4, c1, c2, c3, c4, d1, d2, d3, d4;
   
  
    a1 = m[0][0]; b1 = m[0][1]; 
    c1 = m[0][2]; d1 = m[0][3];
  
    a2 = m[1][0]; b2 = m[1][1]; 
    c2 = m[1][2]; d2 = m[1][3];
  
    a3 = m[2][0]; b3 = m[2][1]; 
    c3 = m[2][2]; d3 = m[2][3];
  
    a4 = m[3][0]; b4 = m[3][1]; 
    c4 = m[3][2]; d4 = m[3][3];
  
    ans = a1 * det3x3( b2, b3, b4, c2, c3, c4, d2, d3, d4)
      - b1 * det3x3( a2, a3, a4, c2, c3, c4, d2, d3, d4)
      + c1 * det3x3( a2, a3, a4, b2, b3, b4, d2, d3, d4)
      - d1 * det3x3( a2, a3, a4, b2, b3, b4, c2, c3, c4);
    return ans;
  }

  void adjoint( double in[4][4], double out[4][4] ) {
    double a1, a2, a3, a4, b1, b2, b3, b4;
    double c1, c2, c3, c4, d1, d2, d3, d4;
  
  
    a1 = in[0][0]; b1 = in[0][1]; 
    c1 = in[0][2]; d1 = in[0][3];
  
    a2 = in[1][0]; b2 = in[1][1]; 
    c2 = in[1][2]; d2 = in[1][3];
  
    a3 = in[2][0]; b3 = in[2][1];
    c3 = in[2][2]; d3 = in[2][3];
  
    a4 = in[3][0]; b4 = in[3][1]; 
    c4 = in[3][2]; d4 = in[3][3];
  
    out[0][0]  =   det3x3( b2, b3, b4, c2, c3, c4, d2, d3, d4);
    out[1][0]  = - det3x3( a2, a3, a4, c2, c3, c4, d2, d3, d4);
    out[2][0]  =   det3x3( a2, a3, a4, b2, b3, b4, d2, d3, d4);
    out[3][0]  = - det3x3( a2, a3, a4, b2, b3, b4, c2, c3, c4);
          
    out[0][1]  = - det3x3( b1, b3, b4, c1, c3, c4, d1, d3, d4);
    out[1][1]  =   det3x3( a1, a3, a4, c1, c3, c4, d1, d3, d4);
    out[2][1]  = - det3x3( a1, a3, a4, b1, b3, b4, d1, d3, d4);
    out[3][1]  =   det3x3( a1, a3, a4, b1, b3, b4, c1, c3, c4);
          
    out[0][2]  =   det3x3( b1, b2, b4, c1, c2, c4, d1, d2, d4);
    out[1][2]  = - det3x3( a1, a2, a4, c1, c2, c4, d1, d2, d4);
    out[2][2]  =   det3x3( a1, a2, a4, b1, b2, b4, d1, d2, d4);
    out[3][2]  = - det3x3( a1, a2, a4, b1, b2, b4, c1, c2, c4);
          
    out[0][3]  = - det3x3( b1, b2, b3, c1, c2, c3, d1, d2, d3);
    out[1][3]  =   det3x3( a1, a2, a3, c1, c2, c3, d1, d2, d3);
    out[2][3]  = - det3x3( a1, a2, a3, b1, b2, b3, d1, d2, d3);
    out[3][3]  =   det3x3( a1, a2, a3, b1, b2, b3, c1, c2, c3);
  }
  
void invert_matrix (double A[4][4], double Ainv[4][4]) {
    int i, j;
    double det ;
  
    adjoint( A, Ainv );
  
    det = det4x4( A );
  
    if ( fabs( det ) < SMALL_NUMBER) {
      fprintf(stderr,"invert_matrix: matrix is singular!");
      return;
    }
  
    for (i=0; i<4; i++)
      for(j=0; j<4; j++)
        Ainv[i][j] = Ainv[i][j] / det;
  }
  

Vec transform_point(const double M[4][4], const Vec& p) {
    double x = M[0][0]*p.x + M[0][1]*p.y + M[0][2]*p.z + M[0][3];
    double y = M[1][0]*p.x + M[1][1]*p.y + M[1][2]*p.z + M[1][3];
    double z = M[2][0]*p.x + M[2][1]*p.y + M[2][2]*p.z + M[2][3];
    return Vec(x, y, z);
}

Vec transform_direction(const double M[4][4], const Vec& d) {
    double x = M[0][0]*d.x + M[0][1]*d.y + M[0][2]*d.z;
    double y = M[1][0]*d.x + M[1][1]*d.y + M[1][2]*d.z;
    double z = M[2][0]*d.x + M[2][1]*d.y + M[2][2]*d.z;
    return Vec(x, y, z);
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        cerr << "Usage: RayTracer.exe <input_file.txt>" << endl;
        return 1;
    }
    parseScene(argv[1]);
    unsigned char* pixels = new unsigned char[3 * imgWidth * imgHeight];
    eye = Vec(0,0,0.15);
    int k = 0;
    for (int j = imgHeight - 1; j >= 0; j--) {
        for (int i = 0; i < imgWidth; i++) {
            double u = leftX + (rightX - leftX) * (i + 0.5) / imgWidth;
            double v = bottomY + (topY - bottomY) * (j + 0.5) / imgHeight;
            Vec dir = Vec(u, v, -nearZ).normalize();
            Vec col = trace(eye, dir, 0);
            col = col * 255.0;
            pixels[k++] = (unsigned char)min(255.0, col.x);
            pixels[k++] = (unsigned char)min(255.0, col.y);
            pixels[k++] = (unsigned char)min(255.0, col.z);
        }
    }

    save_imageP3(imgWidth, imgHeight, outputFile, pixels);
    delete[] pixels;
    return 0;
}
