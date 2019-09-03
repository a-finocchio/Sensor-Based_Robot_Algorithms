// Copyright (C) 2008 Wim Meeussen <meeussen at willowgarage com>
//  
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//  
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//  
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//  


#ifndef __SYSTEM_PDF_HPP__
#define __SYSTEM_PDF_HPP__

#include <bfl/pdf/analyticconditionalgaussian_additivenoise.h>

namespace BFL
{

  class SystemPDF : public AnalyticConditionalGaussianAdditiveNoise{

  public:
    
    SystemPDF( const Gaussian& additiveNoise );
    
    virtual ~SystemPDF();
    
    // redefine virtual functions
    virtual MatrixWrapper::ColumnVector  ExpectedValueGet()    const;
    virtual MatrixWrapper::Matrix        dfGet(unsigned int i) const;
    
  };
  
} // End namespace BFL

#endif //  
