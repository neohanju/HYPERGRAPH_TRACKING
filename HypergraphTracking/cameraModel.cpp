/***************************************************************************
 *   cameraModel.cpp   - description
 *
 *   This program is part of the Etiseo project.
 *
 *   See http://www.etiseo.net  http://www.silogic.fr   
 *
 *   (C) Silogic - Etiseo Consortium
 ***************************************************************************/

#include <math.h>
#include "cameraModel.h"

#import <msxml6.dll>
#include <MsXml6.h>

using namespace Etiseo;

CameraModel::CameraModel()
{
	isInit = false;
}

CameraModel::~CameraModel()
{
}

void CameraModel::internalInit()
{
	double sa;
	double ca;
	double sb;
	double cb;
	double sg;
	double cg;
	
	// compute matrix ...
	sa = sin(mRx);
	ca = cos(mRx);
	sb = sin(mRy);
	cb = cos(mRy);
	sg = sin(mRz);
	cg = cos(mRz);
	
	mR11 = cb * cg;
	mR12 = cg * sa * sb - ca * sg;
	mR13 = sa * sg + ca * cg * sb;
	mR21 = cb * sg;
	mR22 = sa * sb * sg + ca * cg;
	mR23 = ca * sb * sg - cg * sa;
	mR31 = -sb;
	mR32 = cb * sa;
	mR33 = ca * cb;
    
	//compute camera position
	mCposx = -(mTx*mR11 + mTy*mR21 + mTz*mR31);
	mCposy = -(mTx*mR12 + mTy*mR22 + mTz*mR32);
	mCposz = -(mTx*mR13 + mTy*mR23 + mTz*mR33);
	
	isInit = true;
}

void CameraModel::setGeometry(int width, int height, double ncx, double nfx, double dx, double dy, double dpx, double dpy)
{
	mImgWidth = width;
	mImgHeight = height;
	mNcx = ncx;
	mNfx = nfx;
	mDx = dx;
	mDy = dy;
	mDpx = dpx;
	mDpy = dpy; 
	
	isInit = false;
}

void CameraModel::setIntrinsic(double focal, double kappa1, double cx, double cy, double sx)
{
	mFocal = focal;
	mKappa1 = kappa1;
	mCx = cx;
	mCy = cy;
	mSx = sx;
	
	isInit = false;
}
	  
void CameraModel::setExtrinsic(double tx, double ty, double tz, double rx, double ry, double rz)
{
	mTx = tx;
	mTy = ty;
	mTz = tz;
	mRx = rx;
	mRy = ry;
	mRz = rz;
	
	isInit = false;
}

bool CameraModel::fromXml(std::string filename)
{
	// Initialize COM
	::CoInitialize(NULL);
	
	MSXML2::IXMLDOMDocumentPtr pXMLDoc = NULL;
	HRESULT hr = pXMLDoc.CreateInstance(__uuidof(MSXML2::DOMDocument60));
	if(FAILED(hr))
	{
		return false;
	}

	variant_t vResult = pXMLDoc->load(filename.c_str());
	if(false == (bool)vResult)
	{
		return false;
	}

	// first element access
	MSXML2::IXMLDOMElementPtr pRootElement = pXMLDoc->documentElement;

	if((_bstr_t)XML_TAG_CAMERA == pRootElement->nodeName)
	{
		MSXML2::IXMLDOMNodeListPtr pChildNodeList = pRootElement->childNodes;
		for(long idx = 0; idx < pChildNodeList->length; idx++)
		{
			MSXML2::IXMLDOMNodePtr pDOMNode = pChildNodeList->item[idx];
			if((_bstr_t)XML_TAG_GEOMETRY == pDOMNode->nodeName)
			{
				MSXML2::IXMLDOMNamedNodeMapPtr pNameNodeMap = pDOMNode->attributes;
				for(long idx2 = 0; idx2 < pNameNodeMap->length; idx2++)
				{
					MSXML2::IXMLDOMNodePtr pDOMNode2 = pNameNodeMap->item[idx2];
					if((_bstr_t)XML_TAG_WIDTH == pDOMNode2->nodeName)
					{
						mImgWidth = (int)pDOMNode2->nodeValue;
					}
					else if((_bstr_t)XML_TAG_HEIGHT == pDOMNode2->nodeName)
					{
						mImgHeight = (int)pDOMNode2->nodeValue;
					}
					else if((_bstr_t)XML_TAG_NCX == pDOMNode2->nodeName)
					{
						mNcx = (float)pDOMNode2->nodeValue;
					}
					else if((_bstr_t)XML_TAG_NFX == pDOMNode2->nodeName)
					{
						mNfx = (float)pDOMNode2->nodeValue;
					}
					else if((_bstr_t)XML_TAG_DX == pDOMNode2->nodeName)
					{
						mDx = (float)pDOMNode2->nodeValue;
					}
					else if((_bstr_t)XML_TAG_DY == pDOMNode2->nodeName)
					{
						mDy = (float)pDOMNode2->nodeValue;
					}
					else if((_bstr_t)XML_TAG_DPX == pDOMNode2->nodeName)
					{
						mDpx = (float)pDOMNode2->nodeValue;
					}
					else if((_bstr_t)XML_TAG_DPY == pDOMNode2->nodeName)
					{
						mDpy = (float)pDOMNode2->nodeValue;
					}
				}
			}
			else if((_bstr_t)XML_TAG_INTRINSIC == pDOMNode->nodeName)
			{
				MSXML2::IXMLDOMNamedNodeMapPtr pNameNodeMap = pDOMNode->attributes;
				for(long idx2 = 0; idx2 < pNameNodeMap->length; idx2++)
				{
					MSXML2::IXMLDOMNodePtr pDOMNode2 = pNameNodeMap->item[idx2];
					if((_bstr_t)XML_TAG_FOCAL == pDOMNode2->nodeName)
					{
						mFocal = (float)pDOMNode2->nodeValue;
					}
					else if((_bstr_t)XML_TAG_KAPPA1 == pDOMNode2->nodeName)
					{
						mKappa1 = (float)pDOMNode2->nodeValue;
					}
					else if((_bstr_t)XML_TAG_CX == pDOMNode2->nodeName)
					{
						mCx = (float)pDOMNode2->nodeValue;
					}
					else if((_bstr_t)XML_TAG_CY == pDOMNode2->nodeName)
					{
						mCy = (float)pDOMNode2->nodeValue;
					}
					else if((_bstr_t)XML_TAG_SX == pDOMNode2->nodeName)
					{
						mSx = (float)pDOMNode2->nodeValue;
					}
				}
			}
			else if((_bstr_t)XML_TAG_EXTRINSIC == pDOMNode->nodeName)
			{
				MSXML2::IXMLDOMNamedNodeMapPtr pNameNodeMap = pDOMNode->attributes;
				for(long idx2 = 0; idx2 < pNameNodeMap->length; idx2++)
				{
					MSXML2::IXMLDOMNodePtr pDOMNode2 = pNameNodeMap->item[idx2];
					if((_bstr_t)XML_TAG_TX == pDOMNode2->nodeName)
					{
						mTx = (float)pDOMNode2->nodeValue;
					}
					else if((_bstr_t)XML_TAG_TY == pDOMNode2->nodeName)
					{
						mTy = (float)pDOMNode2->nodeValue;
					}
					else if((_bstr_t)XML_TAG_TZ == pDOMNode2->nodeName)
					{
						mTz = (float)pDOMNode2->nodeValue;
					}
					else if((_bstr_t)XML_TAG_RX == pDOMNode2->nodeName)
					{
						mRx = (float)pDOMNode2->nodeValue;
					}
					else if((_bstr_t)XML_TAG_RY == pDOMNode2->nodeName)
					{
						mRy = (float)pDOMNode2->nodeValue;
					}
					else if((_bstr_t)XML_TAG_RZ == pDOMNode2->nodeName)
					{
						mRz = (float)pDOMNode2->nodeValue;
					}
				}
			}
		}
		internalInit();
	}
	pRootElement.Release();
    pXMLDoc.Release();
    CoUninitialize(); 
	
	return isInit;
}

//bool CameraModel::fromXml(std::istream& is)
//{
//	xmlDocPtr doc = xmlReadIO(UtilXml::ReadCallback, 
//				UtilXml::InputCloseCallback, &is,  NULL, NULL, 0);
//	xmlNodePtr node = xmlDocGetRootElement(doc);
//	
//	if (node) {
//			char *temp;
//			if (xmlStrcmp(node->name, XML_TAG_CAMERA) == 0) {
//				
//				temp = (char*)xmlGetProp(node, XML_TAG_NAME);
//				if (temp) 
//					mName = temp;
//			
//				xmlNodePtr child = node->xmlChildrenNode;
//				while (child != NULL) {
//					if (xmlStrcmp(child->name, XML_TAG_GEOMETRY) == 0) {
//						
//						temp = (char*)xmlGetProp(child, XML_TAG_WIDTH);
//						if (temp) {
//							mImgWidth = atoi(temp);
//							xmlFree(temp); 
//						}
//						temp = (char*)xmlGetProp(child, XML_TAG_HEIGHT);
//						if (temp) {
//							mImgHeight = atoi(temp);
//							xmlFree(temp); 
//						}
//						temp = (char*)xmlGetProp(child, XML_TAG_NCX);
//						if (temp) {
//							mNcx = atof(temp);
//							xmlFree(temp); 
//						}
//						temp = (char*)xmlGetProp(child, XML_TAG_NFX);
//						if (temp) {
//							mNfx = atof(temp);
//							xmlFree(temp); 
//						}
//						temp = (char*)xmlGetProp(child, XML_TAG_DX);
//						if (temp) {
//							mDx = atof(temp);
//							xmlFree(temp); 
//						}
//						temp = (char*)xmlGetProp(child, XML_TAG_DY);
//						if (temp) {
//							mDy = atof(temp);
//							xmlFree(temp); 
//						}
//						temp = (char*)xmlGetProp(child, XML_TAG_DPX);
//						if (temp) {
//							mDpx = atof(temp);
//							xmlFree(temp); 
//						}
//						temp = (char*)xmlGetProp(child, XML_TAG_DPY);
//						if (temp) {
//							mDpy = atof(temp);
//							xmlFree(temp); 
//						}
//					} else if (xmlStrcmp(child->name, XML_TAG_INTRINSIC) == 0) {
//						
//						temp = (char*)xmlGetProp(child, XML_TAG_FOCAL);
//						if (temp) {
//							mFocal = atof(temp);
//							xmlFree(temp); 
//						}
//						temp = (char*)xmlGetProp(child, XML_TAG_KAPPA1);
//						if (temp) {
//							mKappa1 = atof(temp);
//							xmlFree(temp); 
//						}
//						temp = (char*)xmlGetProp(child, XML_TAG_CX);
//						if (temp) {
//							mCx = atof(temp);
//							xmlFree(temp); 
//						}
//						temp = (char*)xmlGetProp(child, XML_TAG_CY);
//						if (temp) {
//							mCy = atof(temp);
//							xmlFree(temp); 
//						}
//						temp = (char*)xmlGetProp(child, XML_TAG_SX);
//						if (temp) {
//							mSx = atof(temp);
//							xmlFree(temp); 
//						}
//					} else if (xmlStrcmp(child->name, XML_TAG_EXTRINSIC) == 0) {
//						
//						temp = (char*)xmlGetProp(child, XML_TAG_TX);
//						if (temp) {
//							mTx = atof(temp);
//							xmlFree(temp); 
//						}
//						temp = (char*)xmlGetProp(child, XML_TAG_TY);
//						if (temp) {
//							mTy = atof(temp);
//							xmlFree(temp); 
//						}
//						temp = (char*)xmlGetProp(child, XML_TAG_TZ);
//						if (temp) {
//							mTz = atof(temp);
//							xmlFree(temp); 
//						}
//						temp = (char*)xmlGetProp(child, XML_TAG_RX);
//						if (temp) {
//							mRx = atof(temp);
//							xmlFree(temp); 
//						}
//						temp = (char*)xmlGetProp(child, XML_TAG_RY);
//						if (temp) {
//							mRy = atof(temp);
//							xmlFree(temp); 
//						}
//						temp = (char*)xmlGetProp(child, XML_TAG_RZ);
//						if (temp) {
//							mRz = atof(temp);
//							xmlFree(temp); 
//						}
//					}
//					
//					child = child->next;
//				}
//				
//				internalInit(); 
//			}
//	}
//	return isInit;
//}

void CameraModel::toXml(std::string filename)
{
	::CoInitialize(NULL);

	//read XML
    MSXML2::IXMLDOMDocumentPtr spXMLDoc;
    MSXML2::IXMLDOMElementPtr  spRoot;
    HRESULT hr = spXMLDoc.CreateInstance(__uuidof(MSXML2::DOMDocument60));
 
    if (!SUCCEEDED(hr))
        printf("Unable to create xml file.\n");
    else
    {
        spXMLDoc->raw_createElement((_bstr_t)XML_TAG_CAMERA, &spRoot);
		spXMLDoc->raw_appendChild(spRoot, NULL);

		MSXML2::IXMLDOMElementPtr childGeometryNode;
		childGeometryNode->setAttribute((_bstr_t)XML_TAG_WIDTH, mImgWidth);
		childGeometryNode->setAttribute((_bstr_t)XML_TAG_HEIGHT, mImgHeight);
		childGeometryNode->setAttribute((_bstr_t)XML_TAG_NCX, mNcx);
		childGeometryNode->setAttribute((_bstr_t)XML_TAG_NFX, mNfx);
		childGeometryNode->setAttribute((_bstr_t)XML_TAG_DX, mDx);
		childGeometryNode->setAttribute((_bstr_t)XML_TAG_DY, mDy);
		childGeometryNode->setAttribute((_bstr_t)XML_TAG_DPX, mDpx);
		childGeometryNode->setAttribute((_bstr_t)XML_TAG_DPY, mDpy);
		spRoot->appendChild(childGeometryNode);

		MSXML2::IXMLDOMElementPtr childIntrinsicNode;
		childIntrinsicNode->setAttribute((_bstr_t)XML_TAG_FOCAL, mFocal);
		childIntrinsicNode->setAttribute((_bstr_t)XML_TAG_KAPPA1, mKappa1);
		childIntrinsicNode->setAttribute((_bstr_t)XML_TAG_CX, mCx);
		childIntrinsicNode->setAttribute((_bstr_t)XML_TAG_CY, mCy);
		childIntrinsicNode->setAttribute((_bstr_t)XML_TAG_SX, mSx);
		spRoot->appendChild(childIntrinsicNode);

		MSXML2::IXMLDOMElementPtr childExtrinsicNode;
		childExtrinsicNode->setAttribute((_bstr_t)XML_TAG_TX, mTx);
		childExtrinsicNode->setAttribute((_bstr_t)XML_TAG_TY, mTy);
		childExtrinsicNode->setAttribute((_bstr_t)XML_TAG_TZ, mTz);
		childExtrinsicNode->setAttribute((_bstr_t)XML_TAG_RX, mRx);
		childExtrinsicNode->setAttribute((_bstr_t)XML_TAG_RY, mRy);
		childExtrinsicNode->setAttribute((_bstr_t)XML_TAG_RZ, mRz);
		spRoot->appendChild(childExtrinsicNode);
 
        spXMLDoc->save(filename.c_str());
    }
 
    spRoot.Release();
    spXMLDoc.Release();
    CoUninitialize();
}

//void CameraModel::toXml(std::ostream& os) const
//{
//	XmlOutputHandler	out(os);
//	
//	xmlTextWriterPtr  writer = xmlNewTextWriter(out.xmlOutputBuffer());
//	
//	xmlTextWriterSetIndent (writer, 1);	
//	xmlTextWriterStartDocument(writer, NULL, "UTF-8", NULL);
//
//	xmlTextWriterStartElement(writer, XML_TAG_CAMERA);
//	xmlTextWriterWriteFormatAttribute(writer, XML_TAG_NAME,   "%s", mName.c_str());
//		
//		xmlTextWriterStartElement(writer, XML_TAG_GEOMETRY);
//			xmlTextWriterWriteFormatAttribute(writer, XML_TAG_WIDTH, "%d",	mImgWidth);
//			xmlTextWriterWriteFormatAttribute(writer, XML_TAG_HEIGHT,"%d",	mImgHeight);
//			xmlTextWriterWriteFormatAttribute(writer, XML_TAG_NCX,	 "%lf",	mNcx);
//			xmlTextWriterWriteFormatAttribute(writer, XML_TAG_NFX,   "%lf",	mNfx);
//			xmlTextWriterWriteFormatAttribute(writer, XML_TAG_DX,    "%lf",	mDx);
//			xmlTextWriterWriteFormatAttribute(writer, XML_TAG_DY,	 "%lf",	mDy);
//			xmlTextWriterWriteFormatAttribute(writer, XML_TAG_DPX,   "%lf",	mDpx);
//			xmlTextWriterWriteFormatAttribute(writer, XML_TAG_DPY,   "%lf",	mDpy);
//		xmlTextWriterEndElement(writer); // XML_TAG_GEOMETRY
//
//		
//		xmlTextWriterStartElement(writer, XML_TAG_INTRINSIC);
//			xmlTextWriterWriteFormatAttribute(writer, XML_TAG_FOCAL, "%lf",	mFocal);
//			xmlTextWriterWriteFormatAttribute(writer, XML_TAG_KAPPA1,"%lf",	mKappa1);
//			xmlTextWriterWriteFormatAttribute(writer, XML_TAG_CX,	 "%lf",	mCx);
//			xmlTextWriterWriteFormatAttribute(writer, XML_TAG_CY,    "%lf",	mCy);
//			xmlTextWriterWriteFormatAttribute(writer, XML_TAG_SX,    "%lf",	mSx);
//		xmlTextWriterEndElement(writer); // XML_TAG_INTRINSIC
//	
//		xmlTextWriterStartElement(writer, XML_TAG_EXTRINSIC);
//			xmlTextWriterWriteFormatAttribute(writer, XML_TAG_TX, "%lf",	mTx);
//			xmlTextWriterWriteFormatAttribute(writer, XML_TAG_TY, "%lf",	mTy);
//			xmlTextWriterWriteFormatAttribute(writer, XML_TAG_TZ, "%lf",	mTz);
//			xmlTextWriterWriteFormatAttribute(writer, XML_TAG_RX, "%lf",	mRx);
//			xmlTextWriterWriteFormatAttribute(writer, XML_TAG_RY, "%lf",	mRy);
//			xmlTextWriterWriteFormatAttribute(writer, XML_TAG_RZ, "%lf",	mRz);
//		xmlTextWriterEndElement(writer); // XML_TAG_EXTRINSIC
//	
//	xmlTextWriterEndElement(writer); // XML_TAG_CAMERA
//	
//	xmlTextWriterEndDocument(writer);
//	xmlFreeTextWriter(writer);
//
//}

bool CameraModel::fromDat(std::istream& is, int width, int height)
{
	is >> mNcx;
	is >> mNfx;
	is >> mDx;
	is >> mDy;
	is >> mDpx;
	is >> mDpy;
	is >> mCx;
	is >> mCy;
	is >> mSx;
	is >> mFocal;
	is >> mKappa1;
	is >> mTx;
	is >> mTy;
	is >> mTz;
	is >> mRx;
	is >> mRy;
	is >> mRz;
	
	mImgWidth = width;
	mImgHeight = height;
	
	internalInit();
	
	return isInit;    
    
}

bool CameraModel::imageToWorld(double Xi, double Yi, double Zw, double& Xw, double& Yw)
{
	double Xd;
	double Yd;
	double Xu;
	double Yu;
	double common_denominator;
	
	if (!isInit){ return false; }

	/* convert from image to distorted sensor coordinates */
	Xd = mDpx * (Xi - mCx) / mSx;
	Yd = mDpy * (Yi - mCy);
		
	/* convert from distorted sensor to undistorted sensor plane coordinates */
	distortedToUndistortedSensorCoord (Xd, Yd, Xu, Yu);
		
	/* calculate the corresponding xw and yw world coordinates	 */
	/* (these equations were derived by simply inverting	 */
	/* the perspective projection equations using Macsyma)	 */
	common_denominator = ((mR11 * mR32 - mR12 * mR31) * Yu +
			(mR22 * mR31 - mR21 * mR32) * Xu -
			mFocal * mR11 * mR22 + mFocal * mR12 * mR21);
	
	Xw = (((mR12 * mR33 - mR13 * mR32) * Yu +
		(mR23 * mR32 - mR22 * mR33) * Xu -
		mFocal * mR12 * mR23 + mFocal * mR13 * mR22) * Zw +
		(mR12 * mTz - mR32 * mTx) * Yu +
		(mR32 * mTy - mR22 * mTz) * Xu -
		mFocal * mR12 * mTy + mFocal * mR22 * mTx) / common_denominator;
	
	Yw = -(((mR11 * mR33 - mR13 * mR31) * Yu +
		(mR23 * mR31 - mR21 * mR33) * Xu -
		mFocal * mR11 * mR23 + mFocal * mR13 * mR21) * Zw +
		(mR11 * mTz - mR31 * mTx) * Yu +
		(mR31 * mTy - mR21 * mTz) * Xu -
		mFocal * mR11 * mTy + mFocal * mR21 * mTx) / common_denominator;
		
	return true;
}

void CameraModel::distortedToUndistortedSensorCoord (double Xd, double Yd, double& Xu, double& Yu)
{
	double    distortion_factor;
	
	/* convert from distorted to undistorted sensor plane coordinates */
	distortion_factor = 1 + mKappa1 * (Xd*Xd + Yd*Yd);
	Xu = Xd * distortion_factor;
	Yu = Yd * distortion_factor;
}

bool CameraModel::worldToImage(double Xw, double Yw, double Zw, double& Xi, double& Yi)
{
	bool done = false;
	double xc;
	double yc;
	double zc;
	double Xu;
	double Yu;
	double Xd;
	double Yd;
	
	if (isInit)
	{
		/* convert from world coordinates to camera coordinates */ 
		xc = mR11 * Xw + mR12 * Yw + mR13 * Zw + mTx;
		yc = mR21 * Xw + mR22 * Yw + mR23 * Zw + mTy;
		zc = mR31 * Xw + mR32 * Yw + mR33 * Zw + mTz;
		
		/* convert from camera coordinates to undistorted sensor plane coordinates */
		Xu = mFocal * xc / zc;
		Yu = mFocal * yc / zc;
		
		/* convert from undistorted to distorted sensor plane coordinates */
		undistortedToDistortedSensorCoord (Xu, Yu, Xd, Yd);
		
		/* convert from distorted sensor plane coordinates to image coordinates */
		Xi = Xd * mSx / mDpx + mCx;
		Yi = Yd / mDpy + mCy;
		
		done = true;
	}
	return done;
}

void CameraModel::undistortedToDistortedSensorCoord (double Xu, double Yu, double& Xd, double& Yd)
{
	double Ru;
	double Rd;
	double lambda;
	double c;
	double d;
	double Q;
	double R;
	double D;
	double S;
	double T;
	double sinT;
	double cosT;
	
	if (((Xu == 0) && (Yu == 0)) || (mKappa1 == 0))
	{
		Xd = Xu;
		Yd = Yu;
	}
	else
	{
		Ru = sqrt(Xu*Xu + Yu*Yu);
		
		c = 1.0 / mKappa1;
		d = -c * Ru;
		
		Q = c / 3;
		R = -d / 2;
		D = Q*Q*Q + R*R;
		
		if (D >= 0) 
		{
			/* one real root */
			D = sqrt(D);
			if (R + D > 0)
			{
				S = pow(R + D, 1.0/3.0);
			}
			else
			{
				S = -pow(-R - D, 1.0/3.0);
			}
			
			if (R - D > 0)
			{
				T = pow(R - D, 1.0/3.0);
			}
			else
			{
				T = -pow(D - R, 1.0/3.0);
			}
			
			Rd = S + T;
			
			if (Rd < 0) 
			{
				Rd = sqrt(-1.0 / (3 * mKappa1));
				/*fprintf (stderr, "\nWarning: undistorted image point to distorted image point mapping limited by\n");
				fprintf (stderr, "         maximum barrel distortion radius of %lf\n", Rd);
				fprintf (stderr, "         (Xu = %lf, Yu = %lf) -> (Xd = %lf, Yd = %lf)\n\n", Xu, Yu, Xu * Rd / Ru, Yu * Rd / Ru);*/
			}
		}
		else
		{
			/* three real roots */
			D = sqrt(-D);
			S = pow( sqrt(R*R + D*D) , 1.0/3.0 );
			T = atan2(D, R) / 3;
			sinT = sin(T);
			cosT = cos(T);
			
			/* the larger positive root is    2*S*cos(T)                   */
			/* the smaller positive root is   -S*cos(T) + SQRT(3)*S*sin(T) */
			/* the negative root is           -S*cos(T) - SQRT(3)*S*sin(T) */
			
			Rd = -S * cosT + sqrt(3.0) * S * sinT;	/* use the smaller positive root */
		}
		
		lambda = Rd / Ru;
		
		Xd = Xu * lambda;
		Yd = Yu * lambda;
	}
}

bool CameraModel::undistortedToDistortedImageCoord (double Xfu, double Yfu, double& Xfd, double& Yfd)
{
	bool done = false;
	double Xu;
	double Yu;
	double Xd;
	double Yd;
	
	if (isInit)
	{
		/* convert from image to sensor coordinates */
		Xu = mDpx * (Xfu - mCx) / mSx;
		Yu = mDpy * (Yfu - mCy);
		
		/* convert from undistorted sensor to distorted sensor plane coordinates */
		undistortedToDistortedSensorCoord (Xu, Yu, Xd, Yd);
		
		/* convert from sensor to image coordinates */
		Xfd = Xd * mSx / mDpx + mCx;
		Yfd = Yd / mDpy + mCy;
		
		done = true;
	}
	return done;
}

bool CameraModel::distortedToUndistortedImageCoord (double Xfd, double Yfd, double& Xfu, double& Yfu)
{
	bool done = false;
	double Xd;
	double Yd;
	double Xu;
	double Yu;
	
	if (isInit)
	{
		/* convert from image to sensor coordinates */
		Xd = mDpx * (Xfd - mCx) / mSx;
		Yd = mDpy * (Yfd - mCy);
		
		/* convert from distorted sensor to undistorted sensor plane coordinates */
		distortedToUndistortedSensorCoord (Xd, Yd, Xu, Yu);
		
		/* convert from sensor to image coordinates */
		Xfu = Xu * mSx / mDpx + mCx;
		Yfu = Yu / mDpy + mCy;
	
		done = true;
	}
	return done;
}

bool CameraModel::worldToCameraCoord (double xw, double yw, double zw, double& xc, double& yc, double& zc)
{
	bool done = false;
	
	if (isInit)
	{
		xc = mR11 * xw + mR12 * yw + mR13 * zw + mTx;
		yc = mR21 * xw + mR22 * yw + mR23 * zw + mTy;
		zc = mR31 * xw + mR32 * yw + mR33 * zw + mTz;
		
		done = true;
	}
	return done;
}

bool CameraModel::cameraToWorldCoord (double xc, double yc, double zc, double& xw, double& yw, double& zw)
{
	bool done = false;
	double common_denominator;
	
	if (isInit)
	{
		/* these equations were found by simply inverting the previous routine using Macsyma */
		
		common_denominator = ((mR11 * mR22 - mR12 * mR21) * mR33 +
			(mR13 * mR21 - mR11 * mR23) * mR32 +
			(mR12 * mR23 - mR13 * mR22) * mR31);
			  
		xw = ((mR12 * mR23 -mR13 * mR22) * zc + 
			(mR13 * mR32 - mR12 * mR33) * yc +
			(mR22 * mR33 - mR23 * mR32) * xc +
			(mR13 * mR22 - mR12 * mR23) * mTz +
			(mR12 * mR33 - mR13 * mR32) * mTy +
			(mR23 * mR32 - mR22 * mR33) * mTx) / common_denominator;
			
		yw = -((mR11 * mR23 - mR13 * mR21) * zc +
			(mR13 * mR31 - mR11 * mR33) * yc +
			(mR21 * mR33 - mR23 * mR31) * xc +
			(mR13 * mR21 - mR11 * mR23) * mTz +
			(mR11 * mR33 - mR13 * mR31) * mTy +
			(mR23 * mR31 - mR21 * mR33) * mTx) / common_denominator;
			
		zw = ((mR11 * mR22 - mR12 * mR21) * zc +
			(mR12 * mR31 - mR11 * mR32) * yc +
			(mR21 * mR32 - mR22 * mR31) * xc +
			(mR12 * mR21 - mR11 * mR22) * mTz +
			(mR11 * mR32 - mR12 * mR31) * mTy +
			(mR22 * mR31 - mR21 * mR32) * mTx) / common_denominator;
		
		done = true;
	}
	return done;
}
