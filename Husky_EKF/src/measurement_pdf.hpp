
#ifndef __MEASUREMENT_PDF_HPP__
#define __MEASUREMENT_PDF_HPP__

#include <bfl/pdf/analyticconditionalgaussian_additivenoise.h>

namespace BFL
{

  class MeasurementPDF : public AnalyticConditionalGaussianAdditiveNoise{

  public:
    
    MeasurementPDF( const Gaussian& additiveNoise );
    
    virtual ~MeasurementPDF();
    
    // redefine virtual functions
    virtual MatrixWrapper::ColumnVector  ExpectedValueGet()    const;
    virtual MatrixWrapper::Matrix        dfGet(unsigned int i) const;
    
  };
  
} // End namespace BFL

#endif //  
