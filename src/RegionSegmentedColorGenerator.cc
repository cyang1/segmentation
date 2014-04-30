#include "RegionSegmentedColorGenerator.h"
#include "Events/EventRouter.h"
#include "Events/SegmentedColorFilterBankEvent.h"
#include "Wireless/Wireless.h"
#include "Shared/Profiler.h"
#include "Shared/Config.h"

#include "Shared/debuget.h"

#include <xmmintrin.h>

#ifdef __GNUC__
#  define DECL_ALIGNED(x) __attribute__ ((aligned (x)))
#else
#  define DECL_ALIGNED(x)
#endif

// Inline assembly to check CPU capabilities.
static inline void cpuid(int CPUInfo[4], int InfoType){
  __asm__ __volatile__(
    "cpuid":
    "=a" (CPUInfo[0]),
    "=b" (CPUInfo[1]),
    "=c" (CPUInfo[2]),
    "=d" (CPUInfo[3]) :
    "a" (InfoType), "c" (0)
    );
}

// from http://stackoverflow.com/questions/6121792/how-to-check-if-a-cpu-supports-the-sse3-instruction-set
static inline bool checkSSE3Support()
{
  int info[4];
  cpuid(info, 0);
  int num_ids = info[0];

  if (num_ids > 1) {
    cpuid(info, 1);
    return (info[2] & ((int)1 << 9)) != 0;
  }
  return false;
}

static inline int round( double r ) {
    return (r > 0.0) ? (r + 0.5) : (r - 0.5);
}

RegionSegmentedColorGenerator::RegionSegmentedColorGenerator(unsigned int mysid, FilterBankGenerator* fbg, EventBase::EventTypeID_t tid)
  : FilterBankGenerator("RegionSegmentedColorGenerator",EventBase::visSegmentEGID,mysid,fbg,tid), srcYChan(0), srcUChan(1), srcVChan(2), tmaps(), tmapNames(), numColors(0), colorNames()
{
  //this part is only necessary if you override setNumImages yourself
  if(fbg!=NULL) {
    numLayers=numChannels=0; //this is to force setNumImages to override settings provided by FilterBankGenerator
    setNumImages(fbg->getNumLayers(),fbg->getNumChannels());
  }
}

RegionSegmentedColorGenerator::RegionSegmentedColorGenerator(unsigned int mysid, FilterBankGenerator* fbg, EventBase::EventTypeID_t tid, unsigned int syc, unsigned int suc, unsigned int svc)
  : FilterBankGenerator("RegionSegmentedColorGenerator",EventBase::visSegmentEGID,mysid,fbg,tid), srcYChan(syc), srcUChan(suc), srcVChan(svc), tmaps(), tmapNames(), numColors(0), colorNames()
{
  if(fbg!=NULL) {
    numLayers=numChannels=0; //this is to force setNumImages to override settings provided by FilterBankGenerator
    setNumImages(fbg->getNumLayers(),fbg->getNumChannels());
  }
}

RegionSegmentedColorGenerator::~RegionSegmentedColorGenerator() {
  freeCaches();
  destruct();
  for(unsigned int i=0; i<tmaps.size(); i++)
    delete [] tmaps[i];

  //hashmap::iterator it=colorNames.begin(); //not the way i'd like to iterate, but there seems to be a bug in the current hashmap implementation we're using...
  //for(unsigned int i=0; i<colorNames.size(); it++,i++)
  //free(const_cast<char*>(it->first));
  //colorNames.clear();

  colorNames.clear(); //we're leaking the memory of the names themselves...
}

void
RegionSegmentedColorGenerator::doEvent() {
  if(event->getGeneratorID()==getListenGeneratorID() && event->getSourceID()==getListenSourceID()) {
    SegmentedColorFilterBankEvent segev(this,getGeneratorID(),getSourceID(),EventBase::activateETID,this,getNumColors(),getColors(),&colorNames);
    erouter->postEvent(segev);
    segev.setTypeID(EventBase::statusETID);
    erouter->postEvent(segev);
    segev.setTypeID(EventBase::deactivateETID);
    erouter->postEvent(segev);
  }
}

unsigned int
RegionSegmentedColorGenerator::loadThresholdMap(const std::string& tm_file) {
  const unsigned int size = 1 << (BITS_Y + BITS_U + BITS_V);
  cmap_t * tmap = new cmap_t[size];
  if(!CMVision::LoadThresholdFile(tmap,NUM_Y,NUM_U,NUM_V,config->portPath(tm_file).c_str())) {
    serr->printf("  ERROR: Could not load threshold file '%s'.\n",tm_file.c_str());
    delete [] tmap;
    return -1U;
  }
  if(numColors!=0) {
    //we've already loaded color info, check against that for invalid indexes
    int remapped=CMVision::CheckTMapColors(tmap,NUM_Y,NUM_U,NUM_V,numColors,0);
    if(remapped>0)
      serr->printf("remapped %d colors in threshold file '%s'\n",remapped, tm_file.c_str());
  }

  tmapNames.push_back(tm_file);
  tmaps.push_back(tmap);
  setNumImages(numLayers,tmaps.size());
  return tmaps.size()-1;
}

bool
RegionSegmentedColorGenerator::loadColorInfo(const std::string& col_file) {
  //hashmap::iterator it=colorNames.begin(); //not the way i'd like to iterate, but there seems to be a bug in the current hashmap implementation we're using...
  //for(unsigned int i=0; i<colorNames.size(); it++,i++)
  //free(const_cast<char*>(it->first));
  //colorNames.clear();

  colorNames.clear(); //we're leaking the memory of the names themselves...

  numColors=CMVision::LoadColorInformation(colors,MAX_COLORS,config->portPath(col_file).c_str(),colorNames);
  if(numColors <= 0){
    serr->printf("  ERROR: Could not load colors file:%s\n", col_file.c_str());
    return false;
  }

  //we'd better check the already loaded thresholds (if any) for out of bound indexes
  for(unsigned int i=0; i<tmaps.size(); i++) {
    int remapped=CMVision::CheckTMapColors(tmaps[i],NUM_Y,NUM_U,NUM_V,numColors,0);
    if(remapped>0)
      serr->printf("remapped %d colors in threshold file '%s'\n",remapped, tmapNames[i].c_str());
  }
  return true;
}


unsigned int
RegionSegmentedColorGenerator::getBinSize() const {
  unsigned int used=FilterBankGenerator::getBinSize();
  used+=creatorSize("SegColorImage");
  used+=widths[selectedSaveLayer]*heights[selectedSaveLayer];
  return used;
}

unsigned int
RegionSegmentedColorGenerator::loadBuffer(const char buf[], unsigned int len, const char* filename) {
  unsigned int origlen=len;
  if(!checkInc(FilterBankGenerator::loadBuffer(buf,len,filename),buf,len)) return 0;
  if(!checkCreatorInc("SegColorImage",buf,len)) {
    serr->printf("Unhandled image type for RegionSegmentedColorGenerator: %s",buf+getSerializedSize<unsigned int>());
    return 0;
  } else {
    // actual image
    unsigned int used=widths[selectedSaveLayer]*heights[selectedSaveLayer];
    if(used>len)
      return 0;
    if(images[selectedSaveLayer][selectedSaveChannel]==NULL)
      images[selectedSaveLayer][selectedSaveChannel]=createImageCache(selectedSaveLayer,selectedSaveChannel);
    unsigned char* img=images[selectedSaveLayer][selectedSaveChannel];
    if(img==NULL)
      return 0;
    memcpy(img,buf,used);
    len-=used; buf+=used;

    // color table
    if(!decodeColorsInc(buf,len)) return 0;

    imageValids[selectedSaveLayer][selectedSaveChannel]=true;
    return origlen-len;
  }
}

unsigned int
RegionSegmentedColorGenerator::saveBuffer(char buf[], unsigned int len) const {
  unsigned int origlen=len;
  if(!checkInc(FilterBankGenerator::saveBuffer(buf,len),buf,len)) return 0;
  if(!saveCreatorInc("SegColorImage",buf,len)) return 0;

  // actual image
  unsigned int used=widths[selectedSaveLayer]*heights[selectedSaveLayer];
  if(used>len)
    return 0;
  if(images[selectedSaveLayer][selectedSaveChannel]==NULL) {
    serr->printf("RegionSegmentedColorGenerator::saveBuffer() failed because selected image is NULL -- call selectSaveImage first to make sure it's up to date\n");
    return 0;
  }
  if(!imageValids[selectedSaveLayer][selectedSaveChannel]) {
    serr->printf("RegionSegmentedColorGenerator::saveBuffer() failed because selected image is invalid -- call selectSaveImage first to make sure it's up to date\n");
    return 0;
  }
  unsigned char* img=images[selectedSaveLayer][selectedSaveChannel];
  if(img==NULL)
    return 0;
  memcpy(buf,img,used);
  len-=used; buf+=used;

  // color table
  if(!encodeColorsInc(buf,len)) return 0;

  return origlen-len;
}

bool
RegionSegmentedColorGenerator::encodeColorsInc(char*& buf, unsigned int& len) const {
  if(!encodeInc(numColors,buf,len)) return false;
  for(unsigned int i=0; i<numColors; i++) {
    if(!encodeInc(colors[i].color.red,buf,len)) return false;
    if(!encodeInc(colors[i].color.green,buf,len)) return false;
    if(!encodeInc(colors[i].color.blue,buf,len)) return false;
  }
  return true;
}

bool
RegionSegmentedColorGenerator::decodeColorsInc(const char*& buf, unsigned int& len) {
  if(!decodeInc(numColors,buf,len)) return false;
  for(unsigned int i=0; i<numColors; i++) {
    if(!decodeInc(colors[i].color.red,buf,len)) return false;
    if(!decodeInc(colors[i].color.green,buf,len)) return false;
    if(!decodeInc(colors[i].color.blue,buf,len)) return false;
  }
  return true;
}

void
RegionSegmentedColorGenerator::setDimensions() {
  FilterBankGenerator::setDimensions();
  for(unsigned int i=0; i<numLayers; i++)
    strides[i]=widths[i];
}

void
RegionSegmentedColorGenerator::setNumImages(unsigned int nLayers, unsigned int /*nChannels*/) {
  FilterBankGenerator::setNumImages(nLayers,tmaps.size());
}

unsigned char *
RegionSegmentedColorGenerator::createImageCache(unsigned int layer, unsigned int /*chan*/) const {
  // notice the +1 !!!
  // this is because CMVision is a little naughty and writes an unused, terminator flag at the one-past-end of each row
  // if we didn't add one, this last byte would be beyond the end of the array
  return new unsigned char[widths[layer]*heights[layer]+1];
}

void
RegionSegmentedColorGenerator::nms_val(uint8_t* nms_ptr, uint8_t* s_ptr, int p, int diff) {
  if (p < abs(diff)) {
    return;
  }
  if (s_ptr[p] > s_ptr[p + diff] && s_ptr[p] > s_ptr[p - diff]) {
    nms_ptr[p] = s_ptr[p];
  }
  else {
    nms_ptr[p] = 0;
  }
}

void
RegionSegmentedColorGenerator::trace(int i, int j, uint32_t low, uint8_t *nms, uint8_t *dir, CMVision::image<cmap_t> out) {
  if (out.buf[(i+1) * out.row_stride + (j+1)] == 0) {
    out.buf[(i+1) * out.row_stride + (j+1)] = 255;
    switch (dir[i * out.width + j]) {
      case 0:
        j += 1;
        break;
      case 45:
        j -= 1;
        i += 1;
        break;
      case 90:
        i += 1;
        break;
      case 135:
        i += 1;
        j += 1;
        break;
      default:
        break;
    }
    if (i >= 1 && i < out.height-2 && j >= 1 && j < out.width-2 &&
      nms[i * out.width + j] >= low) {
      trace(i, j, low, nms, dir, out);
    }
  }
}

// from OpenCV source, adapted for special purpose use
void
RegionSegmentedColorGenerator::bilateralFilter(CMVision::image_yuv<const cmap_t> in,
    CMVision::image_yuv<cmap_t> out,
    int d,
    double sigma_color,
    double sigma_space)
{
  // Images being bilateral filtered must have 3 channels.
  const int CHANNELS = 3

  double gauss_color_coeff = -0.5 / (sigma_color*sigma_color);
  double gauss_space_coeff = -0.5 / (sigma_space*sigma_space);

  int radius = std::max(d / 2, 1);
  d = radius * 2 + 1;

  CMVision::image_yuv<cmap_t> temp;
  copyMakeReflect101Border(in, temp, radius);

  std::vector<float> _color_weight(CHANNELS * 256);
  std::vector<float> _space_weight(d*d);
  std::vector<int> _space_ofs(d*d);
  float* color_weight = &_color_weight[0];
  float* space_weight = &_space_weight[0];
  int* space_ofs = &_space_ofs[0];

  // initialize color-related bilateral filter coefficients
  for (int i = 0; i < 256 * CHANNELS; i++)
    color_weight[i] = (float)std::exp(i*i*gauss_color_coeff);

  // initialize space-related bilateral filter coefficients
  int maxk = 0;
  for (int i = -radius; i <= radius; i++) {
    for (int j = -radius; j <= radius; j++) {
      double r = std::sqrt((double)i*i + (double)j*j);
      if (r > radius)
        continue;
      space_weight[maxk] = (float)std::exp(r*r*gauss_space_coeff);
      space_ofs[maxk++] = (int)(i*temp.row_stride + j);
    }
  }

  // finished initialization, time to perform actual filter
  int DECL_ALIGNED(16) buf[4];
  float DECL_ALIGNED(16) bufSum[4];
  static const int DECL_ALIGNED(16) bufSignMask[] = { 0x80000000, 0x80000000, 0x80000000, 0x80000000 };
  bool haveSSE3 = checkSSE3Support();

  for (int i = 0; i < in.height; i++) {
    const cmap_t* sptr_y = &temp.buf_y[(i+radius)*temp.row_stride + radius],
                  sptr_u = &temp.buf_u[(i+radius)*temp.row_stride + radius],
                  sptr_v = &temp.buf_v[(i+radius)*temp.row_stride + radius];

    cmap_t* dptr_y = &dst.buf_y[i*dst.row_stride],
            dptr_u = &dst.buf_u[i*dst.row_stride],
            dptr_v = &dst.buf_v[i*dst.row_stride];

    for (int j = 0; j < in.width; j++) {
      float sum_b = 0, sum_g = 0, sum_r = 0, wsum = 0;
      int b0 = sptr_y[j], g0 = sptr_u[j], r0 = sptr_v[j];
      int k = 0;

      if (haveSSE3) {
        const __m128i izero = _mm_setzero_si128();
        const __m128 _b0 = _mm_set_ps1(static_cast<float>(b0));
        const __m128 _g0 = _mm_set_ps1(static_cast<float>(g0));
        const __m128 _r0 = _mm_set_ps1(static_cast<float>(r0));
        const __m128 _signMask = _mm_load_ps((const float*)bufSignMask);

        for (; k <= maxk - 4; k += 4) {
          // Technically a gather would be better here, but need AVX
          // for that.
          const char b_data[4] = { sptr_y[j + space_ofs[k]],
            sptr_y[j + space_ofs[k + 1]],
            sptr_y[j + space_ofs[k + 2]],
            sptr_y[j + space_ofs[k + 3]] };

          const char g_data[4] = { sptr_u[j + space_ofs[k]],
            sptr_u[j + space_ofs[k + 1]],
            sptr_u[j + space_ofs[k + 2]],
            sptr_u[j + space_ofs[k + 3]] }

          const char r_data[4] = { sptr_v[j + space_ofs[k]],
            sptr_v[j + space_ofs[k + 1]],
            sptr_v[j + space_ofs[k + 2]],
            sptr_v[j + space_ofs[k + 3]] }

          const int* const b_ptr = reinterpret_cast<const int*>(&b_data);
          const int* const g_ptr = reinterpret_cast<const int*>(&g_data);
          const int* const r_ptr = reinterpret_cast<const int*>(&r_data);

          __m128 _b = _mm_cvtepi32_ps(_mm_unpacklo_epi16(_mm_unpacklo_epi8(_mm_cvtsi32_si128(b_ptr), izero), izero));
          __m128 _g = _mm_cvtepi32_ps(_mm_unpacklo_epi16(_mm_unpacklo_epi8(_mm_cvtsi32_si128(g_ptr), izero), izero));
          __m128 _r = _mm_cvtepi32_ps(_mm_unpacklo_epi16(_mm_unpacklo_epi8(_mm_cvtsi32_si128(r_ptr), izero), izero));

          __m128 bt = _mm_andnot_ps(_signMask, _mm_sub_ps(_b, _b0));
          __m128 gt = _mm_andnot_ps(_signMask, _mm_sub_ps(_g, _g0));
          __m128 rt = _mm_andnot_ps(_signMask, _mm_sub_ps(_r, _r0));

          bt = _mm_add_ps(rt, _mm_add_ps(bt, gt));
          _mm_store_si128((__m128i*)buf, _mm_cvtps_epi32(bt));

          __m128 _w = _mm_set_ps(color_weight[buf[3]], color_weight[buf[2]],
            color_weight[buf[1]], color_weight[buf[0]]);
          __m128 _sw = _mm_loadu_ps(space_weight + k);

          _w = _mm_mul_ps(_w, _sw);
          _b = _mm_mul_ps(_b, _w);
          _g = _mm_mul_ps(_g, _w);
          _r = _mm_mul_ps(_r, _w);

          _w = _mm_hadd_ps(_w, _b);
          _g = _mm_hadd_ps(_g, _r);

          _w = _mm_hadd_ps(_w, _g);
          _mm_store_ps(bufSum, _w);

          wsum += bufSum[0];
          sum_b += bufSum[1];
          sum_g += bufSum[2];
          sum_r += bufSum[3];
        }
      }

      for (; k < maxk; k++)
      {
        int idx = j + space_ofs[k];
        int b = sptr_y[idx], g = sptr_u[idx], r = sptr_v[idx];
        float w = space_weight[k] * color_weight[std::abs(b - b0) +
          std::abs(g - g0) + std::abs(r - r0)];
        sum_b += b*w; sum_g += g*w; sum_r += r*w;
        wsum += w;
      }
      wsum = 1.f / wsum;
      b0 = round(sum_b*wsum);
      g0 = round(sum_g*wsum);
      r0 = round(sum_r*wsum);
      dptr_y[j] = (uchar)b0; dptr_u[j] = (uchar)g0; dptr_v[j] = (uchar)r0;
    }
  }
}

void
RegionSegmentedColorGenerator::canny(CMVision::image<const cmap_t> in, CMVision::image<cmap_t> out, unsigned int low, unsigned int high) {
  // assert(in.buf != NULL && out.buf != NULL);
  // assert(in.width == out.width && in.height == out.height);
  // assert(low < high);

  uint8_t *sobel, *dir, *nms;
  sobel = (uint8_t*)malloc(sizeof(uint8_t) * in.width * in.height);
  dir = (uint8_t*)malloc(sizeof(uint8_t) * in.width * in.height);
  nms = (uint8_t*)malloc(sizeof(uint8_t) * in.width * in.height);

  /* SOBEL */
  int dx[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
  int dy[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};

  for (int i = 1; i < in.height - 1; i++) {
    for (int j = 1; j < in.width - 1; j++) {
      int index = i*in.width + j;
      int sum_x = 0, sum_y = 0;
      for (int m = -1; m <= 1; m++) {
        for (int n = -1; n <= 1; n++) {
          cmap_t pixel = in.buf[(i+m)*in.row_stride + (j+n)];
          sum_x += pixel*dx[m+1][n+1];
          sum_y += pixel*dy[m+1][n+1];
        }
      }
      int sum = sqrt(sum_x*sum_x + sum_y*sum_y);
      sobel[index] = (sum > 255) ? 255 : sum;

      /*
       * Estimate the direction of the sobel gradient, in degrees.
       */
      if (sum_x == 0 && sum_y == 0) {
        dir[index] = 0;
      }
      else if (sum_x == 0) {
        dir[index] = 90;
      }
      else {
        double abs_div = abs(sum_y / (double) sum_x);
        if (abs_div > 2.41421356237) {  //atan2(2.4...) ~= 67.5 deg
          dir[index] = 0;
        }
        else if (abs_div > 0.414213562373) {    //atan(0.4...) ~= 22.5 deg
          if ((sum_y / sum_x) > 0) {
            dir[index] = 45;
          }
          else {
            dir[index] = 135;
          }
        }
        else {
          dir[index] = 90;
        }
      }
    }
  }
  /* END SOBEL */

  /* NON MAX SUPPRESSION */

  for (int i = 1; i < in.height - 1; i++) {
    for (int j = 1; j < in.width - 1; j++) {
      int p = i*in.width + j;
      switch (dir[p]) {
        case 0:     // N/S
          nms_val(nms, sobel, p, in.width);
          break;
        case 45:    // NW/SE
          nms_val(nms, sobel, p, in.width + 1);
          break;
        case 90:    // W/E
          nms_val(nms, sobel, p, 1);
          break;
        case 135:   // NE/SW
          nms_val(nms, sobel, p, in.width - 1);
          break;
        default:
          break;
      }
    }
  }
  /* END NON MAX SUPPRESSION */

  /* HYSTERESIS */

  for (int i = 1; i < out.height - 1; i++) {
    for (int j = 1; j < out.width - 1; j++) {
      if (nms[i * out.width + j] >= high) {
        trace(i, j, low, nms, dir, out);
      }
    }
  }

  /* END HYSTERESIS */
}

void
RegionSegmentedColorGenerator::yuvtolab(CMVision::image_yuv<const cmap_t> in, CMVision::image_yuv<cmap_t> out) {
  // assert(in.width == out.width && in.height == out.height);
  // to rgb first
  for (int row = 0; row < in.width; row++) {
    for (int col = 0; col < in.height; col++) {
      const cmap_t in_y = in.buf_y[row*in.row_stride + col*in.col_stride],
                   in_u = in.buf_u[row*in.row_stride + col*in.col_stride],
                   in_v = in.buf_v[row*in.row_stride + col*in.col_stride];
      double r = in_y + 1.13983*in_v, g = in_y - 0.39465*in_u - 0.58060*in_v, b = in_y + 2.03211*in_v;
      double x = 0.412453*r + 0.357580*g + 0.180423*b,
             y = 0.212671*r + 0.715160*g + 0.072169*b,
             z = 0.019334*r + 0.119193*g + 0.950227*b;
      double x_n = 0.9642, y_n = 1.0000, z_n = 0.8249;
      double xx_n = x/x_n, yy_n = y/y_n, zz_n = z/z_n;
      cmap_t L, A, B;
      if (yy_n > 0.008856) {
        L = 116 * pow(yy_n, 1./3) - 16;
      }
      else {
        L= 903.3 * yy_n;
      }
      double fxx_n = (xx_n > 0.008856) ? pow(xx_n, 1./3) : 7.787 * xx_n + 16./116,
             fyy_n = (yy_n > 0.008856) ? pow(yy_n, 1./3) : 7.787 * yy_n + 16./116,
             fzz_n = (zz_n > 0.008856) ? pow(zz_n, 1./3) : 7.787 * zz_n + 16./116;
      A = 500 * (fxx_n - fyy_n);
      B = 200 * (fyy_n - fzz_n);
      out.buf_y[row*out.row_stride + col*out.col_stride] = L;
      out.buf_u[row*out.row_stride + col*out.col_stride] = A;
      out.buf_v[row*out.row_stride + col*out.col_stride] = B;
    }
  }
}

void
RegionSegmentedColorGenerator::calcImage(unsigned int layer, unsigned int chan) {
  if(tmaps.size()==0)
    throw NoThresholdException();
  PROFSECTION("RegionSegmentedColorGenerator::calcImage(...)",*mainProfiler);
  CMVision::image_yuv<const cmap_t> img(src->getImage(layer, srcYChan), src->getImage(layer, srcUChan), src->getImage(layer, srcVChan),
      getWidth(layer), getHeight(layer), src->getStride(layer), src->getIncrement(layer));

  CMVision::image_yuv<cmap_t> lab_img(createImageCache(layer,1), createImageCache(layer,1), createImageCache(layer,1), img.width, img.height, img.width, 1);
  CMVision::image_yuv<cmap_t> smoothed_lab_img(createImageCache(layer,1), createImageCache(layer,1), createImageCache(layer,1), img.width, img.height, img.width, 1);
  CMVision::image<const cmap_t> gray_img(img.buf_y, img.width, img.height, img.width);
  CMVision::image<cmap_t> mask(new unsigned char[(widths[layer]+2)*(heights[layer]+2)+1], img.width + 2, img.height, img.width + 2);

  yuvtolab(img, lab_img);
  bilateralFilter(lab_img, smoother_lab_img, 10, 20, 5);
  canny(gray_img, mask, 40, 90);
  // floodfill

  CMVision::ThresholdImageYUVPlanar<cmap_t,CMVision::image_yuv<const cmap_t>,const cmap_t,BITS_Y,BITS_U,BITS_V>(images[layer][chan],img,tmaps[chan]);
  imageValids[layer][chan]=true;
}

/*! @file
 * @brief Implements RegionSegmentedColorGenerator, which generates FilterBankEvents indexed color images based on a color threshold file
 * @author alokl (Creator)
 * @author ejt (reorganized)
 */