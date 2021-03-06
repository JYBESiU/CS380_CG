#define NO_GLR_MACROS
#include "GLRenderer.h"
#include <stdlib.h>
#include <glm/gtc/type_ptr.hpp>

int cnt = 0;

FrameBuffer::FrameBuffer()
    : _pixels( 0 ), _zbuffer( 0 ), _width( 0 ), _height( 0 ) {
    SetSize( 256, 256 ); // start out with something
}

FrameBuffer::~FrameBuffer() {
    delete[] _pixels;
    delete[] _zbuffer;
}

void FrameBuffer::SetSize( int w, int h ) {
    if( w != _width || h != _height ) {
        delete[] _pixels;
        delete[] _zbuffer;

        _width = w;
        _height = h;
        _pixels = new glm::vec4[_width * _height];
        _zbuffer = new float[_width * _height];
    }
}

void FrameBuffer::GetSize( int &w, int &h ) {
    w = _width;
    h = _height;
}

void FrameBuffer::Display( bool setDepth ) {
    checkForGLError( "FrameBuffer::Display() - begin"  );

    glMatrixMode( GL_PROJECTION );  glPushMatrix();
    glLoadIdentity();
    glMatrixMode( GL_MODELVIEW );  glPushMatrix();
    glLoadIdentity();
    int viewport[4];
    glGetIntegerv( GL_VIEWPORT, viewport );
    glViewport( 0, 0, _width, _height );
    int depthTest = glIsEnabled( GL_DEPTH_TEST ) ;

    glDisable( GL_DEPTH_TEST );
    glRasterPos2f( -1, -1 );
    glDrawPixels( _width, _height, GL_RGBA, GL_FLOAT, _pixels );
    if( setDepth )
        glDrawPixels( _width, _height, GL_DEPTH_COMPONENT, GL_FLOAT, _zbuffer );

    if( depthTest )
        glEnable( GL_DEPTH_TEST );
    glViewport( viewport[0], viewport[1], viewport[2], viewport[3] );
    glMatrixMode( GL_PROJECTION );
    glPopMatrix();
    glMatrixMode( GL_MODELVIEW );
    glPopMatrix();

    checkForGLError( "FrameBuffer::Display() - end"  );
}

GLRenderer::GLRenderer()
    :
    _useSoftwareRendering( false ),
    _primitiveType( 0 ),
    _captureVertices( false ),
    _lastTextureID( -1 ),
    _displayFrameBuffer( false )
{}

GLRenderer::~GLRenderer() {
    if( texture.data ) {
        delete[] texture.data[0];
        delete texture.data;
    }
}

GLRenderer *theGLRenderer = NULL;

//------------------------------------------------------------------------------
void GLRenderer::SetGlobalInstance( GLRenderer *gle ) {
    theGLRenderer = gle;
}

//------------------------------------------------------------------------------
GLRenderer *GLRenderer::GetGlobalInstance() {
    return theGLRenderer;
}

//------------------------------------------------------------------------------
// A function that implements glBegin() as software.
// Mode is only implemented about GL_POLYGON, GL_TRIANGLES, GL_QUARDS.
void GLRenderer::Begin( GLenum mode ) {
    if( _useSoftwareRendering &&
            ( mode == GL_POLYGON || mode == GL_TRIANGLES || mode == GL_QUADS ) ) {
        _primitiveType = mode;
        _captureVertices = true;
        GetState();
        _vertices.clear();
    } else {
        // let GL handle it
        _primitiveType = 0;
        _captureVertices = false;
        glBegin( mode );
    }
}

//------------------------------------------------------------------------------
void GLRenderer::End() {
    if( !_useSoftwareRendering || !_captureVertices ) {
        glEnd();
        return;
    }

    // send individual polygons down the rendering pipeline
    int stride;
    switch( _primitiveType ) {
    case GL_POLYGON:   stride = ( int )_vertices.size(); break;
    case GL_TRIANGLES: stride = 3; break;
    case GL_QUADS:     stride = 4; break;
    default:
        printf( "GLRenderer::glEnd() - Unknown primitive type.\n" );
        return;
    }

    vector<GLVertex> poly;
    int pos = 0;


    while( pos < ( int )_vertices.size() ) {
        poly.clear();
        for( int i = 0; i < stride && pos < ( int )_vertices.size(); i++ )
            poly.push_back( _vertices[pos++] );
        processPolygon( poly );
    }

    _captureVertices = false;
}


//------------------------------------------------------------------------------
void GLRenderer::processPolygon( vector<GLVertex> &verts ) {
    int i;

    if( verts.size() < 3 ) {
        printf( "GLRenderer::processPolygon() - Not enough vertices.\n" );
        return;
    }

    CoordinateSystem coords = MODEL;

    // transform to eye coordinates
    for( i = 0; i < ( int )verts.size(); i++ ) {
        verts[i].position = modelviewMatrix * verts[i].position;

        // Normals transform like planes - by the inverse. If the modelview matrix is orthonormal
        // we don't have to take the inverse. We can just use the transpose. If we multiply the normal
        // on the right instead of the right, it has the same effect as multiplying by the transpose.
        // verts[i].normal = modelviewMatrix.multMatrixVec3( verts[i].normal );
        verts[i].normal = glm::normalize( glm::mat3( modelviewMatrix ) * verts[i].normal );
    }
    coords = EYE;

    // lighting
    if( lightingEnabled && !ComputeLighting( verts ) ) {
        passVerticesToGL( verts, coords, false );
        return;
    }

    // transform to clip coordinates
    for( i = 0; i < ( int )verts.size(); i++ ) {
        verts[i].position = projectionMatrix * verts[i].position;
    }
    coords = CLIP;

    // clipping
    vector<GLVertex> clippedVerts;
    if( !ClipPolygon( verts, clippedVerts ) ) {
        passVerticesToGL( verts, coords, false );
        return;
    }

    // divide by w and perform viewport transformation to window coordinates
    glm::vec4 scale( 0.5 * viewport[2], 0.5 * viewport[3], 0.5, 1 );
    glm::vec4 translate( viewport[0] + 0.5 * viewport[2], viewport[1] + 0.5 * viewport[3], 0.5, 0 );
    for( i = 0; i < ( int )clippedVerts.size(); i++ ) {
        clippedVerts[i].position *= scale / clippedVerts[i].position[3];
        clippedVerts[i].position += translate;
    }
    coords = WINDOW;

    // triangulation
    vector<GLVertex> triVerts;
    if( !TriangulatePolygon( clippedVerts, triVerts ) ) {
        passVerticesToGL( clippedVerts, coords, false );
        return;
    }

    
    // rasterization
    for( i = 0; i < ( int )triVerts.size(); i += 3 ) {
        if( !RasterizeTriangle( &triVerts[i] ) ) {
            passVerticesToGL( triVerts, coords, true );
            return;
        }
        cnt += 1;
    }

    _displayFrameBuffer = true;
}

//------------------------------------------------------------------------------
void GLRenderer::passVerticesToGL( vector<GLVertex> &verts, CoordinateSystem coords,
                                   bool triangles  ) {
    checkForGLError( "GLRenderer::passVerticesToGL() - begin" );


    if( coords >= CLIP ) {
        // we've already applied PROJECTION matrix - set it to identity
        glMatrixMode( GL_PROJECTION );
        glPushMatrix();
        glLoadIdentity();
        glMatrixMode( GL_MODELVIEW );

        // also disable lighting because we would not have gotten this far
        // if we hadn't calculated it already
        glDisable( GL_LIGHTING );
    }
    if( coords >= EYE ) {
        // we've already applied MODELVIEW matrix - set it to identity
        glPushMatrix();
        glLoadIdentity();
    }
    if( coords == WINDOW ) {
        int w, h;
        frameBuffer.GetSize( w, h );

        // draw to the whole window so we can see if we are clipping correctly
        glPushAttrib( GL_VIEWPORT_BIT );
        glViewport( 0, 0, w, h );
        glOrtho( 0, w, 0, h, 0, -1 );
    }


    // send vertices to GL
    GLenum mode = triangles ? GL_TRIANGLES : GL_POLYGON;
    glBegin( mode );
    for( int i = 0; i < ( int )verts.size(); i++ ) {
        glColor4fv( glm::value_ptr( verts[i].color ) );
        glNormal3fv( glm::value_ptr( verts[i].normal ) );
        glTexCoord2fv( verts[i].texCoord );
        glVertex4fv( glm::value_ptr( verts[i].position ) );
    }
    glEnd();

    // Restore matrices
    if( coords >= CLIP ) {
        glMatrixMode( GL_PROJECTION );
        glPopMatrix();
        glMatrixMode( GL_MODELVIEW );
    }
    if( coords >= EYE )
        glPopMatrix();

    // Restore viewport
    if( coords == WINDOW )
        glPopAttrib();

    if( lightingEnabled )
        glEnable( GL_LIGHTING );

    checkForGLError( "GLRenderer::passVerticesToGL() - end" );
}

//------------------------------------------------------------------------------
//  Read state of OpenGL, and save it to class GLRenderer's member variable
void GLRenderer::GetState() {
    lightingEnabled = glIsEnabled( GL_LIGHTING );
    depthTestEnabled = glIsEnabled( GL_DEPTH_TEST );
    textureEnabled = glIsEnabled( GL_TEXTURE_2D );
    cullFaceEnabled = glIsEnabled( GL_CULL_FACE );

    glGetIntegerv( GL_VIEWPORT, viewport );
    glGetFloatv( GL_PROJECTION_MATRIX, glm::value_ptr( projectionMatrix ) );
    glGetFloatv( GL_MODELVIEW_MATRIX, glm::value_ptr( modelviewMatrix ) );
    glGetFloatv( GL_CURRENT_COLOR, glm::value_ptr( _currentGLVertex.color ) );
    glGetFloatv( GL_CURRENT_NORMAL, glm::value_ptr( _currentGLVertex.normal ) );
    float texCoord[4];
    glGetFloatv( GL_CURRENT_TEXTURE_COORDS, texCoord );
    _currentGLVertex.texCoord[0] = texCoord[0];
    _currentGLVertex.texCoord[1] = texCoord[1];
    if( lightingEnabled ) {
        GetMaterial( GL_FRONT, material );
        GetLight( GL_LIGHT0, light );
    }
    if( textureEnabled )
        GetTexture( texture );

    frameBuffer.SetSize( glutGet( GLUT_WINDOW_WIDTH ), glutGet( GLUT_WINDOW_HEIGHT ) );
}

//------------------------------------------------------------------------------
void GLRenderer::GetMaterial( GLenum face, Material &m ) {
    glGetMaterialfv( face, GL_AMBIENT, glm::value_ptr( m.ambient ) );
    glGetMaterialfv( face, GL_DIFFUSE, glm::value_ptr( m.diffuse ) );
    glGetMaterialfv( face, GL_SPECULAR, glm::value_ptr( m.specular ) );
    glGetMaterialfv( face, GL_SHININESS, &m.shininess );
}

//------------------------------------------------------------------------------
void GLRenderer::GetLight( GLenum which, Light &l ) {
    glGetLightfv( which, GL_POSITION, glm::value_ptr( l.position ) );
    glGetLightfv( which, GL_AMBIENT, glm::value_ptr( l.ambient ) );
    glGetLightfv( which, GL_DIFFUSE, glm::value_ptr( l.diffuse ) );
    glGetLightfv( which, GL_SPECULAR, glm::value_ptr( l.specular ) );
    l.enabled = glIsEnabled( which );
}

//------------------------------------------------------------------------------
void GLRenderer::GetTexture( Texture &t ) {
    glGetIntegerv( GL_TEXTURE_BINDING_2D, &t.id );
    glGetTexParameteriv( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, &t.wrapS );
    glGetTexParameteriv( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, &t.wrapT );
    glGetTexParameteriv( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, &t.minFilter );
    glGetTexParameteriv( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, &t.magFilter );

    // pull data only once
    if( t.id == _lastTextureID )
        return;

    if( t.data ) {
        delete[] t.data[0]; // data
        delete[] t.data;
    }
    t.levels = 0;
    glGetTexLevelParameteriv( GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &t.width );
    glGetTexLevelParameteriv( GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &t.height );
    int size = t.width * t.height;
    int w = t.width;
    int h = t.height;
    while( w > 0 && h > 0 ) {
        t.levels++;
        size += w * h;
        glGetTexLevelParameteriv( GL_TEXTURE_2D, t.levels, GL_TEXTURE_WIDTH, &w );
        glGetTexLevelParameteriv( GL_TEXTURE_2D, t.levels, GL_TEXTURE_HEIGHT, &h );
    }

    if( size ) {
        t.data = new glm::vec4*[t.levels];
        t.data[0] = new glm::vec4[size];
        int pos = 0;
        w = t.width;
        h = t.height;
        for( int i = 0; i < t.levels; i++ ) {
            t.data[i] = t.data[0] + pos;
            glGetTexImage( GL_TEXTURE_2D, i, GL_RGBA, GL_FLOAT, t.data[i] );
            pos += w * h;
            w >>= 1;
            h >>= 1;
            if( w < 1 ) w = 1;
            if( h < 1 ) h = 1 ;
        }
    }
    _lastTextureID = t.id;
}

//------------------------------------------------------------------------------
void GLRenderer::Display( bool setDepth ) {
    if( _useSoftwareRendering && _displayFrameBuffer )
        frameBuffer.Display( setDepth );

    if (cullFaceEnabled) {
        printf("Culling... -> ");
    }
    else {
        printf("No cull... -> ");

    }
    printf("triangle: %d\n", cnt);

}


//------------------------------------------------------------------------------
void GLRenderer::Clear( GLbitfield mask ) {
    _displayFrameBuffer = false;
    if( _useSoftwareRendering ) {
        float clearColor[4];
        glGetFloatv( GL_COLOR_CLEAR_VALUE, clearColor );
        int w = glutGet( GLUT_WINDOW_WIDTH );
        int h = glutGet( GLUT_WINDOW_HEIGHT );
        frameBuffer.SetSize( w, h );
        if( mask & GL_COLOR_BUFFER_BIT ) {
            for( int y = 0; y < h; y++ )
                for( int x = 0; x < w; x++ )
                    frameBuffer.SetPixel( x, y, *( glm::vec4 * )clearColor );
        }
        if( mask & GL_DEPTH_BUFFER_BIT ) {
            for( int y = 0; y < h; y++ )
                for( int x = 0; x < w; x++ )
                    frameBuffer.SetDepth( x, y, 1.0 );
        }
    }

    cnt = 0;

    glClear( mask );
}


//------------------------------------------------------------------------------
void GLRenderer::Vertex4f( float x, float y, float z, float w ) {
    if( _captureVertices ) {
        _currentGLVertex.position[0] = x;
        _currentGLVertex.position[1] = y;
        _currentGLVertex.position[2] = z;
        _currentGLVertex.position[3] = w;
        _vertices.push_back( _currentGLVertex );
    } else
        glVertex4f( x, y, z, w );
}



//------------------------------------------------------------------------------
void GLRenderer::TexCoord2f( float x, float y ) {
    if( _captureVertices ) {
        _currentGLVertex.texCoord[0] = x;
        _currentGLVertex.texCoord[1] = y;
    } else
        glTexCoord2f( x, y );
}

//------------------------------------------------------------------------------
void GLRenderer::Normal3f( float x, float y, float z ) {
    if( _captureVertices ) {
        _currentGLVertex.normal[0] = x;
        _currentGLVertex.normal[1] = y;
        _currentGLVertex.normal[2] = z;
    } else
        glNormal3f( x, y, z );
}

//------------------------------------------------------------------------------
void GLRenderer::Color4f( float r, float g, float b, float a ) {
    if( _captureVertices ) {
        _currentGLVertex.color[0] = r;
        _currentGLVertex.color[1] = g;
        _currentGLVertex.color[2] = b;
        _currentGLVertex.color[3] = a;
    } else
        glColor4f( r, g, b, a  );
}


//------------------------------------------------------------------------------
void checkForGLError( const char *msg ) {
    GLenum errCode;
    const GLubyte *errStr;
    if ( ( errCode = glGetError() ) != GL_NO_ERROR ) {
        errStr = gluErrorString( errCode );
        fprintf( stderr, "OpenGL ERROR: %s: %s\n", errStr, msg );
    }
}
