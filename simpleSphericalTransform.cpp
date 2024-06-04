// simpleSphericalTransform.C
/*! \class simpleSphericalTransform simpleSphericalTransform.C

   This class implements a plug-in to the DDImage library that modified the image
   as if it was over the surface of a sphere. It copies the function of the spherical transform
   in latlong from-to mode but processing the image much faster as it is more simple.

   \author Jorge Jacob García-Asenjo
   \date June 4th, 2024 File creation.
 */

// Standard plug-in include files.

static const char* const CLASS = "simpleSphericalTransform";
static const char* const HELP = "Transforms the input image as if\n"
                                "it was the surface of a sphere,\n"
                                "you can change the position of the poles.";

#include <stdio.h>
#include "DDImage/Iop.h"
#include "DDImage/NukeWrapper.h"
#include "DDImage/Row.h"
#include "DDImage/Tile.h"
#include "DDImage/Pixel.h"
#include "DDImage/Filter.h"
#include "DDImage/Knobs.h"
#include "DDImage/Vector2.h"
#include "DDImage/DDMath.h"
#include "DDImage/MultiTileIop.h"
#include "DDImage/MultiTileIopEngineDefinitions.h"
#include "DDImage/Sampler.h"

using namespace DD::Image;


class simpleSphericalTransform : public MultiTileIop
{
    double center[2];
    double PI;
    int _width;
    int _height;
    Vector2 p;
    Vector2 q;
    Filter filter;    

public:

  simpleSphericalTransform (Node* node) : MultiTileIop (node)
  {
    center[0] = center[1] = 0;  }

  ~simpleSphericalTransform () override { }

  void _validate(bool) override
  {
    filter.initialize();
    copy_info();
  }

  void _request(int x, int y, int r, int t, ChannelMask channels, int count) override
  {
    ChannelSet c1(channels);
    in_channels(0, c1);
    input0().request(input0().info().x(),
                     input0().info().y(),
                     input0().info().r(),
                     input0().info().t(),
                     c1,
                     count * 2);
  }

  Iop* inputToRead() const override;

  template<class TileType> inline void doEngine(int y, int x, int r, ChannelMask channels, Row& row);

  mFnDDImageMultiTileIop_DeclareFunctions_engine(int y, int x, int r, ChannelMask m, Row& row);

  void knobs ( Knob_Callback f ) override
  {
    XY_knob(f, &center[0], "center");
    Tooltip(f, "Set the pole of the pole of the sphere in the projection.");
    filter.knobs(f);
    Tooltip(f, "Set the filter type for the spherical transformation.");
  }

  const char* Class() const override { return CLASS; }
  const char* node_help() const override { return HELP; }
  static const Iop::Description description;
};

static Iop* simpleSphericalTransformCreate(Node* node)
{
  return (new NukeWrapper (new simpleSphericalTransform(node)))->noMix()->noMask();
}
const Iop::Description simpleSphericalTransform::description ( CLASS, nullptr, simpleSphericalTransformCreate );

Iop* simpleSphericalTransform::inputToRead() const
{
  return &input0();
}

template<class TileType> void simpleSphericalTransform::doEngine ( int y, int x, int r, ChannelMask channels, Row& out )
{
  double PI = 3.14159265359;
  int _width = info().r(); // Get width of image
  int _height = info().t(); // Get height of image
  if (aborted()) {
    return;
  }
  foreach(z, channels) out.writable(z);
  InterestRatchet interestRatchet;
  Pixel pixel(channels);
  pixel.setInterestRatchet(&interestRatchet);

  std::vector<SamplePosition> samplePositions;
  samplePositions.reserve(r - x + 1);

        for (int X = x; X < r; X++) { // For each horizontal pixel within the row	
                if ( aborted() ) {
                    return;
                }
                //uvMap
                Vector2 p((float(X) + 0.5f) / float(_width) , (float(y) + 0.5f) / float(_height));

                //uvMap to radians
                Vector2 q((p.x) * (2*PI) , (p.y) * (PI) - (PI/2));     

                //Radians to Normals
                float Xrot = cos(q[1]) * cos(q[0] + radians(-(center[0]/float(_width) * 360) +90));
                float Yrot = sin(q[1]);
                float Zrot = cos(q[1]) * sin(q[0] + radians(-(center[0]/float(_width) * 360) +90));

                float Xnorm = Xrot;
                float Ynorm = Yrot*cos(radians(center[1]/float(_height) *180))  + Zrot*-sin(radians(center[1]/float(_height) *180));
                float Znorm = Yrot*sin(radians(center[1]/float(_height) *180))  + Zrot*cos(radians(center[1]/float(_height) *180));
                //90 degrees rotation
                Vector3 rotNorm(Znorm,  Ynorm,  -Xnorm);
                //Normal to radians again
                Vector2 normRad(rotNorm[2] > 0 ? atan2(rotNorm[2],rotNorm[0]) : (2*PI) + atan2(rotNorm[2],rotNorm[0]), asin(rotNorm[1]));
                //Radians to uvMap again
                Vector2 finalUV(  normRad[0]/(2*PI)  ,  (normRad[1] + (PI/2) )/PI);
                //Place pixels in correspondent new position
                Vector2 newPos(floor(finalUV[0] * _width) + 0.5f,  floor(finalUV[1] * _height) + 0.5f);

                Vector2 du (1, -1);
                Vector2 dv (-1, 1);

               samplePositions.emplace_back(newPos, du, dv, X);
  }
  Sampler sampler(&input0(), input0().requestedBox(), channels, &filter, Sampler::eEdgeFromIop, &interestRatchet, true);

  for (auto& samplePosition : samplePositions) {
    sampler.sample(samplePosition, pixel);
    for (auto z: channels) {
      ((float*)(out[z]))[samplePosition.x] = pixel[z];
    }
  }
}

mFnDDImageMultiTileIop_DefineFunctions_engine(simpleSphericalTransform)
