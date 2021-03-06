#ifndef _MYGL_H_
#define _MYGL_H_

#include "GLRenderer.h"

typedef vector<GLVertex>::size_type size_type;

//
// MAIN CLASS TO IMPLEMENT FOR SOFTWARE RENDERING PIPELINE
//

//==============================================================================
class MyGL : public GLRenderer
//==============================================================================
{
  public:
    MyGL();
    ~MyGL();

    // Methods to turn the various stages on/off
    bool GetDoTriangulate()         { return _doTriangulate; }
    void SetDoTriangulate( bool v ) { _doTriangulate = v; }
    bool GetDoRasterize()           { return _doRasterize; }
    void SetDoRasterize( bool v )   { _doRasterize = v; }

  protected:
    // The following functions are called by GLRenderer::processPolygon().
    virtual bool TriangulatePolygon( const vector<GLVertex> &polygonVerts,
                                     vector<GLVertex> &triangleVerts );
    virtual bool RasterizeTriangle( GLVertex triVerts[3] );
    bool EdgeCheck(float x, float y, glm::vec3 e01, glm::vec3 e12, glm::vec3 e20, bool edge);
    void InterpolateColor(int x, int y, glm::vec3 linInterpR, glm::vec3 linInterpG, glm::vec3 linInterpB, glm::vec3 linInterpA);
    int FindYMax(GLVertex verts[3]);
    int FindXMax(GLVertex verts[3]);

  private:
    bool _doTriangulate;
    bool _doRasterize;
};

#endif // _MYGL_H_