#include "RegionRegionSegmentedColorGenerator.h"
#include "Events/EventRouter.h"
#include "Events/SegmentedColorFilterBankEvent.h"
#include "Wireless/Wireless.h"
#include "Shared/Profiler.h"
#include "Shared/Config.h"

#include "Shared/debuget.h"

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
RegionSegmentedColorGenerator::trace(int i, int j, uint32_t low, uint8_t *nms, uint8_t *dir, CMVision::image<const cmap_t> out) {
    if (out->buf[i * out.row_stride + j] == 0) {
        out->buf[i * out.row_stride + j] = 255;
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
        if (i >= 0 && i < rows && j >= 0 && j < out.width &&
            nms[i * out.width + j] >= low) {
            trace(i, j, low, nms, dir, out);
        }
    }
}

void
RegionSegmentedColorGenerator::canny(CMVision::image<const cmap_t> in, CMVision::image<const cmap_t> out, unsigned int low, unsigned int high) {
  assert(in->buf != NULL && out->buf != NULL);
  assert(in.width == out.width && in.height == out.height);
  assert(low < high);

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
                  cmap_t pixel = in->buf[(i+m)*in.row_stride + (j+n)];
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
              dir_ptr[index] = 0;
          }
          else if (sum_x == 0) {
              dir_ptr[index] = 90;
          }
          else {
              double abs_div = abs(sum_y / (double) sum_x);
              if (abs_div > 2.41421356237) {  //atan2(2.4...) ~= 67.5 deg
                  dir_ptr[index] = 0;
              }
              else if (abs_div > 0.414213562373) {    //atan(0.4...) ~= 22.5 deg
                  if ((sum_y / sum_x) > 0) {
                      dir_ptr[index] = 45;
                  }
                  else {
                      dir_ptr[index] = 135;
                  }
              }
              else {
                  dir_ptr[index] = 90;
              }
          }
      }
  }
  /* END SOBEL */

  /* NON MAX SUPPRESSION */

  for (int i = 1; i < in.height - 1; i++) {
      for (int j = 1; j < in.width - 1; j++) {
          int p = i*in.width + j;
          switch (dir_ptr[p]) {
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
  assert(in.width == out.width && in.height == out.height);
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
      double xx_n = x/x_n, yy_n, y/y_n, zz_n = z/z_n;
      cmap_t l, a, b;
      if (yy_n > 0.008856) {
        l = 116 * pow(yy_n, 1./3) - 16;
      }
      else {
        l = 903.3 * yy_n;
      }
      double fxx_n = (xx_n > 0.008856) ? pow(xx_n, 1./3), 7.787 * xx_n + 16./116,
             fyy_n = (yy_n > 0.008856) ? pow(yy_n, 1./3), 7.787 * yy_n + 16./116,
             fzz_n = (zz_n > 0.008856) ? pow(zz_n, 1./3), 7.787 * zz_n + 16./116;
      a = 500 * (fxx_n - fyy_n);
      b = 200 * (fyy_n - fzz_n);
      out.buf_y[row*out.row_stride + col*out.col_stride] = l;
      out.buf_u[row*out.row_stride + col*out.col_stride] = a;
      out.buf_v[row*out.row_stride + col*out.col_stride] = b;
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

  // blur
  // convert to LAB, greyscale
  CMVision::image<const cmap_t> gray_img(img.buf_y, width, height, width);
  CMVision::image<cmap_t> canny_img(createImageCache(layer), width, height, width);
  canny(gray_img, canny_img, 40, 90);
  // floodfill

  CMVision::ThresholdImageYUVPlanar<cmap_t,CMVision::image_yuv<const cmap_t>,const cmap_t,BITS_Y,BITS_U,BITS_V>(images[layer][chan],img,tmaps[chan]);
  imageValids[layer][chan]=true;
}

/*! @file
 * @brief Implements RegionSegmentedColorGenerator, which generates FilterBankEvents indexed color images based on a color threshold file
 * @author alokl (Creator)
 * @author ejt (reorganized)
 */
