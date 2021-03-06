
////////////////////////////////////////////////////////////////////////////////
//
// GLRenderer
//
//   The GLRenderer intercepts a small subset of the standard GL calls and
//   sends primitives down a software rendering pipeline. Preprocessor macros
//   are used to replace the GL calls with calls to an instance of GLRenderer.
//   If software rendering mode is enabled, the primitives that would have been
//   sent to GL are instead sent down the software rendering pipeline consisting
//   of the following stages:
//
//     1. Lighting (if GL_LIGHTING is enabled)
//     2. Clipping
//     3. Triangulation
//     4. Rasterization
//
//   To aid in debugging, any stage can abort the rendering process and
//   the primitives processed up to that stage will be sent to GL.
//
//   Note that only immediate mode rendering is captured. GLVertex
//   arrays are currently not supported.The following primitives are currently
//   support by GLRenderer:
//     GL_TRIANGLES
//     GL_QUADS
//     GL_POLYGON
//
//   To use GLRenderer:
//     1. Place this header file in any files where rendering is performed
//     2. Create a subclass of GLRenderer that implements the various stages
//     3. Set the global GLRenderer instance by calling
//        GLRenderer::SetGlobalInstance() with a pointer to an object of
//        your GLRenderer subclass.
//     4. Call the Display() at the end of the frame
//
////////////////////////////////////////////////////////////////////////////////
#ifndef _GLRENDERER_H_
#define _GLRENDERER_H_

#include <GL/glut.h>
#include <vector>
using std::vector;
#include <assert.h>
#include <glm/glm.hpp>
#include <glm/glm.hpp>

//==============================================================================
class GLVertex
//==============================================================================
{
  public:
    glm::vec4 position;
    glm::vec4 color;
    glm::vec3 normal;
    float texCoord[2];
};

//==============================================================================
class Material
//
// Encapsulates state for GL materials
//==============================================================================
{
  public:
    glm::vec4 ambient;
    glm::vec4 diffuse;
    glm::vec4 specular;
    float shininess;
};

//==============================================================================
class Light
//
// Encapsulates state for GL lights
//==============================================================================
{
  public:
    glm::vec4 ambient;
    glm::vec4 diffuse;
    glm::vec4 specular;
    glm::vec4 position;
    int   enabled;
};

//==============================================================================
class Texture
//
// Encapsulates state for GL textures
//==============================================================================
{
  public:
    int id;
    int minFilter;
    int magFilter;
    int wrapS;
    int wrapT;
    int width;
    int height;
    int levels;
    glm::vec4 **data; // pointer to an array of pointers for each level

    Texture() : id( 0 ), data( 0 ), levels( 0 ), width( 0 ), height( 0 ) {}

    // Retrieves the pixel at (x,y) from the indicate mipmap level
    glm::vec4 GetPixel( int x, int y, int level = 0 ) {
        assert( x >= 0 && x < ( width >> level ) && y >= 0 && y < ( height >> level ) );
        return data[level][y * ( width >> level ) + x];
    }

    // Sets the pixel at (x,y) in the indicate mipmap level
    void SetPixel( int x, int y, int level, const glm::vec4 &color ) {
        assert( x >= 0 && x < ( width >> level ) && y >= 0 && y < ( height >> level ) );
        data[level][y * ( width >> level ) + x] = color;
    }
};

//==============================================================================
class FrameBuffer
//
// A simple frame buffer class
//==============================================================================
{
  public:
    FrameBuffer();
    ~FrameBuffer();

    // Copies the frame buffer to the screen.
    //   setDepth - indicates whether to copy the depth buffer as well
    void Display( bool setDepth = true );

    glm::vec4& GetPixel(int x, int y) {
        assert(x >= 0 && x < _width && y >= 0 && y < _height);
        return _pixels[y * _width + x];
    }
    void SetPixel(int x, int y, const glm::vec4& color) {
        assert(x >= 0 && x < _width && y >= 0 && y < _height);
        _pixels[y * _width + x] = color;
    }

    float GetDepth(int x, int y) {
        assert(x >= 0 && x < _width && y >= 0 && y < _height);
        return _zbuffer[y * _width + x];
    }

    void  SetDepth(int x, int y, float depth) {
        assert(x >= 0 && x < _width && y >= 0 && y < _height);
        _zbuffer[y * _width + x] = depth;
    }

    void SetSize( int w, int h );
    void GetSize( int &w, int &h );

  protected:
    glm::vec4 *_pixels;
    float *_zbuffer;
    int _width, _height;
};


//==============================================================================
class GLRenderer
//==============================================================================
{
  public:
    GLRenderer();
    virtual ~GLRenderer();

    // Copies the frame buffer to the screen. Only copies the frame buffer if
    // triangles have been rasterized since the last call to Clear()
    //   setDepth - indicates whether to copy the depth buffer
    void Display( bool setDepth = false );

    // When SoftwareRendering = false, GL commands are passed to hardware. Otherwise,
    // the software rendering functions of this class are invoked for each primitive.
    void SetSoftwareRendering( bool v ) { _useSoftwareRendering = v; }
    bool GetSoftwareRendering()         { return _useSoftwareRendering; }

    // Sets the global instace of GLRenderer through which GL calls are routed
    static void SetGlobalInstance( GLRenderer *gle );
    static GLRenderer *GetGlobalInstance();

    //-----------------------------------
    // GL replacement functions
    //-----------------------------------
    void Clear( GLbitfield mask );
    void Begin( GLenum mode );
    void End();
    void Vertex4f( float x, float y, float z, float w );
    void TexCoord2f( float x, float y );
    void Normal3f( float x, float y, float z );
    void Color4f( float r, float g, float b, float a );



  protected:
    //-----------------------------------
    // Rendering pipeline functionality
    //-----------------------------------

	

	

    // Computes lighting for polygon vertices
    //   verts - polygon vertices with position and normals transformed to eye
    //           coordinates. The lighting is stored in the color of the vertices
    // Return false to abort the pipeline    
	bool ComputeLighting(vector<GLVertex> &verts) { return false; }

    // Clips a polygon to the view frustum
    //   vertsIn  - polygon vertices in clip coordinates
    //   vertsOut - contains clipped polygon vertices upon return
    // Return false to abort the pipeline
	bool ClipPolygon(const vector<GLVertex> &vertsIn, vector<GLVertex> &vertsOut) {
		vertsOut = vertsIn;
		return true;
	}

    // Triangulates a polygon
    //   polygonVerts  - polygon vertices
    //   triangleVerts - contains triangle vertices upon return. Every three
    //                   vertices defines a triangle.
    // Return false to abort the pipeline
    virtual bool TriangulatePolygon( const vector<GLVertex> &polygonVerts,
                                     vector<GLVertex> &triangleVerts )
    { return false; }

    // Rasterizes a triangle
    //   verts - an array of the three triangle vertices
    // Return false to abort the pipeline
    virtual bool RasterizeTriangle( GLVertex triVerts[3] ) { return false; }


    // Sets internal variables for lighting, materials, textures, and other
    // GL state. Override this function to capture additional state.
    virtual void GetState();

    void GetMaterial( GLenum face, Material &m );
    void GetLight( GLenum which, Light &l );
    void GetTexture( Texture &t );

    // State needed for rasterization
    int lightingEnabled;
    int depthTestEnabled;
    int textureEnabled;
    int cullFaceEnabled;
    int  viewport[4];
    glm::mat4 projectionMatrix;
    glm::mat4 modelviewMatrix;
    Material material;
    Light    light;
    Texture  texture;
    FrameBuffer frameBuffer;

  private:
    enum CoordinateSystem {
        MODEL,
        EYE,
        CLIP,
        WINDOW
    };

    // Called to send a polygon down the pipeline
    void  processPolygon( vector<GLVertex> &poly );

    // Renders partially processed vertices with OpenGL.
    //   coords   - used to indicate at what point in the pipeline the vertices are
    //   triangle - Indicates whether to interpret verts as a single polygon or
    //              as a set of triangles
    void  passVerticesToGL( vector<GLVertex> &verts, CoordinateSystem coords, bool triangles = false );


    GLenum _primitiveType;        // type of primitive being stored in _vertices
    bool _captureVertices;        // controls vertex accumulation
    GLVertex _currentGLVertex;        // accumulates state changes to normal, texCoord, and color
    vector<GLVertex> _vertices;     // accumulates vertices between glBegin()/glEnd()
    bool  _useSoftwareRendering;
    bool _displayFrameBuffer;     // indicates that triangle have been rasterized
    //   and that the framebuffer should be copied on
    //   a call to Display()
    int  _lastTextureID;
};


void checkForGLError( const char *msg );

// GL function replacement macros

#ifndef NO_GLR_MACROS
#define glClear(mask)           theGLRenderer->Clear(mask)
#define glBegin(mode)           theGLRenderer->Begin(mode)
#define glEnd()                 theGLRenderer->End()
#define glVertex4f(x, y, z, w ) theGLRenderer->Vertex4f(x,y,z,w)
#define glVertex3f(x, y, z )    theGLRenderer->Vertex4f( x, y, z, 1 )
#define glVertex2f(x, y )       theGLRenderer->Vertex4f( x, y, 0, 1 )
#define glVertex4fv( v )        theGLRenderer->Vertex4f( v[0], v[1], v[2], v[3] )
#define glVertex3fv( v )        theGLRenderer->Vertex4f( v[0], v[1], v[2], 1 )
#define glVertex2fv( v )        theGLRenderer->Vertex4f( v[0], v[1], 0, 1 )
#define glTexCoord2f( x, y )    theGLRenderer->TexCoord2f( x, y )
#define glTexCoord2fv( v )      theGLRenderer->TexCoord2f( v[0], v[1] )
#define glNormal3f( x, y, z )   theGLRenderer->Normal3f( x, y, z )
#define glNormal3fv( v )        theGLRenderer->Normal3f( v[0], v[1], v[2] )
#define glColor4f( r, g, b, a ) theGLRenderer->Color4f( r, g, b, a)
#define glColor3f( r, g, b )    theGLRenderer->Color4f( r, g, b, 1 )
#define glColor4fv( v )         theGLRenderer->Color4f( v[0], v[1], v[2], v[3] )
#define glColor3fv( v )         theGLRenderer->Color4f( v[0], v[1], v[2], 1 )

#define glVertex4d(x, y, z, w ) theGLRenderer->Vertex4f(x,y,z,w)
#define glVertex3d(x, y, z )    theGLRenderer->Vertex4f( x, y, z, 1 )
#define glVertex2d(x, y )       theGLRenderer->Vertex4f( x, y, 0, 1 )
#define glVertex4dv( v )        theGLRenderer->Vertex4f( v[0], v[1], v[2], v[3] )
#define glVertex3dv( v )        theGLRenderer->Vertex4f( v[0], v[1], v[2], 1 )
#define glVertex2dv( v )        theGLRenderer->Vertex4f( v[0], v[1], 0, 1 )
#define glTexCoord2d( x, y )    theGLRenderer->TexCoord2f( x, y )
#define glTexCoord2dv( v )      theGLRenderer->TexCoord2f( v[0], v[1] )
#define glNormal3d( x, y, z )   theGLRenderer->Normal3f( x, y, z )
#define glNormal3dv( v )        theGLRenderer->Normal3f( v[0], v[1], v[2] )
#define glColor4d( r, g, b, a ) theGLRenderer->Color4f( r, g, b, a)
#define glColor3d( r, g, b )    theGLRenderer->Color4f( r, g, b, 1 )
#define glColor4dv( v )         theGLRenderer->Color4f( v[0], v[1], v[2], v[3] )
#define glColor3dv( v )         theGLRenderer->Color4f( v[0], v[1], v[2], 1 )

// the global GLRenderer through which to route GL calls
extern GLRenderer *theGLRenderer;
#endif // NO_GLR_MACROS


#endif // _GLRENDERER_H_
