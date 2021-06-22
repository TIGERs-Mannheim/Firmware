#include "mex.h"
#include <stdint.h>
#include <vector>
#include <map>
#include <list>
#include <string>
#include <memory>
#include <fstream>
#include <cstring>

using namespace std;

#define MAX_MSG_SIZE 512
#define CHUNK_HEADER_SIZE 4
#define CTYPE_MESSAGE_DESC 0x8000
#define CTYPE_ELEMENT_DESC 0x8001

#define BUF_SIZE 16384
char readBuf[BUF_SIZE];

/** COBS Stuff */
#define COBSMaxStuffedSize(size) (size+size/208+1)
typedef enum
{
	Unused = 0x00, 		/* Unused (framing character placeholder) */
	DiffZero = 0x01, 	/* Range 0x01 - 0xD1:                     */
	DiffZeroMax = 0xD1, /* n-1 explicit characters plus a zero    */
	Diff = 0xD2, 		/* 209 explicit characters, no added zero */
	RunZero = 0xD3, 	/* Range 0xD3 - 0xDF:                     */
	RunZeroMax = 0xDF, 	/* 3-15 zeroes                            */
	Diff2Zero = 0xE0, 	/* Range 0xE0 - 0xFF:                     */
	Diff2ZeroMax = 0xFF,/* 0-31 explicit characters plus 2 zeroes */
} StuffingCode;
#define isDiff2Zero(X) (X >= Diff2Zero)
#define isRunZero(X)   (X >= RunZero && X <= RunZeroMax)
#define ConvertZP (Diff2Zero - DiffZero) // = 0xDF = 223
#define MaxConvertible (Diff2ZeroMax - ConvertZP) // = 0x20 = 32

int16_t COBSDecode(const uint8_t *pIn, uint32_t sizeIn,
		uint8_t *pOut, uint32_t sizeOut, uint32_t* pBytesWritten)
{
	const uint8_t* pOutOrig = pOut;
	const uint8_t *pEnd = pIn + sizeIn;
	const uint8_t *pLimit = pOut + sizeOut;

	if(sizeIn == 0)
		return 0;

	while (pIn < pEnd)
	{
		int32_t z, c = *pIn++; // c = code, z = zeros

		if (c == Diff)
		{
			z = 0;
			c--;
		}
		else if (isRunZero(c))
		{
			z = c & 0xF;
			c = 0;
		}
		else if (isDiff2Zero(c))
		{
			z = 2;
			c &= 0x1F;
		}
		else
		{
			z = 1;
			c--;
		}

		while (--c >= 0)
		{
			if(pOut < pLimit)
				*pOut = *pIn;
			++pOut;
			++pIn;
		}

		while (--z >= 0)
		{
			if(pOut < pLimit)
				*pOut = 0;
			++pOut;
		}
	}

	if(pBytesWritten)
		*pBytesWritten = pOut-pOutOrig-1;

	if (pOut >= pLimit)
		return 1;

	return 0;
}

#define MAX_ENCODED_MSG_SIZE COBSMaxStuffedSize(MAX_MSG_SIZE)


class Converter;
class MessageContainer;

map<uint8_t, shared_ptr<Converter>> elementTypes;

class Message
{
public:
	
private:
	vector<double> data;
};

class Converter
{
public:
	virtual double toDouble(void* pData) =0;
	virtual size_t getSize() =0;
};

template <typename T>
double toDouble(void* pData)
{
	return (double)(*((T*)pData));
}

template <typename T>
class StdConverter : public Converter
{
public:
	double toDouble(void* pData)
	{
		return (double)(*((T*)pData));
	}
	
	size_t getSize()
	{
		return sizeof(T);
	}
};

class MessageContainer
{
public:
	void parseHeader(void* pData);
	void parseElement(void* pData);
	void parseData(void* pData);
	
	uint16_t getId() { return id; }
	
	uint16_t id;
	string name;
	uint16_t numElements;
    uint32_t timestampRollovers;
	
	vector<string> names;
	vector<string> units;
	vector<string> descs;
	list<vector<double>> data;
	
	vector<shared_ptr<Converter>> elements;	// typeID
};

void MessageContainer::parseHeader(void* pData)
{
	uint8_t* pBuf = (uint8_t*)pData;
	
	id = *((uint16_t*)&pBuf[4]);
	numElements = *((uint16_t*)&pBuf[6]);
	name = string((char*)&pBuf[8]);
	
	names.push_back("t");
	units.push_back("s");
	descs.push_back("Time");
	
//	mexPrintf("ID: 0x%04X\nEl: %hu\nName: %s\n", id, numElements, name.c_str());
}

void MessageContainer::parseElement(void* pData)
{
	uint8_t* pBuf = (uint8_t*)pData;

	int type = pBuf[4];
	string name((char*)&pBuf[5]);
	string unit((char*)&pBuf[5+name.length()+1]);
	string desc((char*)&pBuf[5+name.length()+unit.length()+2]);
	
	names.push_back(name);
	units.push_back(unit);
	descs.push_back(desc);
	
	elements.push_back(elementTypes[type]);
	
//	mexPrintf("Type: %hhu\t%s\t%s\t%s\n", type, name.c_str(), unit.c_str(), desc.c_str());
}

void MessageContainer::parseData(void* pData)
{
    if(numElements != elements.size())
    {
        mexWarnMsgIdAndTxt("MATLAB:loadLogFile:incompleteMsg",
                    "Ignoring data for incomplete message %s.", name.c_str());
        
        return;
    }
    
	uint8_t* pBuf = (uint8_t*)pData;
	pBuf += 4;
	
	uint32_t timestamp = *((uint32_t*)pBuf);
	pBuf += 4;
	
	vector<double> row;
	row.resize(numElements+1);
	row[0] = timestamp/1000000.0 + timestampRollovers*4294.967296;
	
  	if(!data.empty())
	{
        if(row[0] < 600.0 && data.back()[0] > 3694.967296)
        {
            // timestamp rollover of 32bit [us]
            timestampRollovers++;
            row[0] += 4294.967296;
        }
        else if(row[0] < data.back()[0]) // TODO: handle 32bit [us] rollover
		{
			mexWarnMsgIdAndTxt("MATLAB:loadLogFile:invalidTimestamp",
						"Element with non-increasing timestamp detected. (%s)", name.c_str());
			return;
		}
	}
	
	for(uint16_t i = 0; i < numElements; i++)
	{
		shared_ptr<Converter> pConv = elements[i];
		
		row[i+1] = pConv->toDouble(pBuf);
		pBuf += pConv->getSize();
	}
	
	data.push_back(row);
}

void mexFunction(	int nlhs, mxArray *plhs[],
					int nrhs, const mxArray *prhs[])
{
	map<uint16_t, shared_ptr<MessageContainer>> logData;
	elementTypes.clear();

	uint8_t decodeBuf[MAX_ENCODED_MSG_SIZE];

	if(nrhs != 1)
	{
		mexErrMsgIdAndTxt( "MATLAB:loadLogFile:invalidNumInputs",
			"One input argument required.");
	}

	if(nlhs != 1)
	{
		mexErrMsgIdAndTxt( "MATLAB:loadLogFile:invalidNumOutputs",
			"One output argument required.");
	}

	if(!(mxIsChar(prhs[0])))
	{
		mexErrMsgIdAndTxt( "MATLAB:loadLogFile:inputNotString",
			"Input must be of type string.\n.");
	}
	
	elementTypes[0] = make_shared<StdConverter<uint8_t>>();
	elementTypes[1] = make_shared<StdConverter<int8_t>>();
	elementTypes[2] = make_shared<StdConverter<uint16_t>>();
	elementTypes[3] = make_shared<StdConverter<int16_t>>();
	elementTypes[4] = make_shared<StdConverter<uint32_t>>();
	elementTypes[5] = make_shared<StdConverter<int32_t>>();
	elementTypes[6] = make_shared<StdConverter<float>>();
	elementTypes[7] = make_shared<StdConverter<double>>();
	
	char* pFilename = mxArrayToString(prhs[0]);
	
	mexPrintf("Opening: %s\n", pFilename);
	
	ifstream file(pFilename, ifstream::in | ifstream::binary);
	if(file.fail())
	{
		mexErrMsgIdAndTxt( "MATLAB:loadLogFile:invalidFile",
		"Cannot open the specified file.");
	}
	
	mxFree(pFilename);
	
	shared_ptr<MessageContainer> pCurMsg;
    
    uint32_t totalSize = 0;
    
    file.seekg(0, ifstream::end);
    uint32_t fileSize = file.tellg();
    file.seekg(0, ifstream::beg);
	
	while(!file.eof())
	{
		file.read(readBuf, BUF_SIZE);
		streamsize bytesRead = file.gcount();
		int readPos = 0;
		
		while(readPos < bytesRead)
		{
			char* pNull = (char*)memchr(readBuf+readPos, 0, BUF_SIZE-readPos);
            if(pNull == 0)
            {
                file.seekg(-(bytesRead-readPos), ifstream::cur);
                break;
            }
            
            int lengthRaw = pNull-readBuf-readPos+1;
            
            uint32_t bytesWritten;
            int16_t result = COBSDecode((uint8_t*)(readBuf+readPos), lengthRaw-1, decodeBuf, MAX_ENCODED_MSG_SIZE, &bytesWritten);
            
            readPos += lengthRaw;
            
            uint16_t type = *((uint16_t*)&decodeBuf[0]);
			uint16_t length = *((uint16_t*)&decodeBuf[2]);
            
            if(bytesWritten != length)
            {
                mexWarnMsgIdAndTxt("MATLAB:loadLogFile:lengthMismatch",
						"COBS length not equal to message length. (%u != %u)", bytesWritten, length);
                
                continue;
            }
            
            totalSize += bytesWritten;

			switch(type)
			{
				case CTYPE_MESSAGE_DESC:
				{
					pCurMsg = make_shared<MessageContainer>();
					pCurMsg->parseHeader(decodeBuf);
                    if(logData.find(pCurMsg->getId()) == logData.end())
                    {
                        logData[pCurMsg->getId()] = pCurMsg;
                    }
				}
				break;
				case CTYPE_ELEMENT_DESC:
				{
					pCurMsg->parseElement(decodeBuf);
				}
				break;
				default:
				{
                    auto it = logData.find(type);
                    if(it == logData.end())
                        mexWarnMsgIdAndTxt("MATLAB:loadLogFile:invalidElement", "Type not found (0x%04X)", type);
                    else
                        it->second->parseData(decodeBuf);
				}
				break;
			}
			
//			mexPrintf("Type: 0x%04X\nLength: %hu\n", type, length);
		}
		
//		mexPrintf("Read %d bytes\n", bytesRead);
	}
	
	file.close();
    
    mexPrintf("Data was compressed by: %5.2f%%\n", 100-((float)fileSize)/((float)totalSize)*100);
	
	// complete content is now in memory
	
	vector<const char*> msgNames;
	
	mexPrintf("Message containers: %u\n", logData.size());
	
	for(map<uint16_t, shared_ptr<MessageContainer>>::iterator iter = logData.begin();
		iter != logData.end(); iter++)
	{
		mexPrintf("%s: %u\n", iter->second->name.c_str(), iter->second->data.size());
		
		msgNames.push_back(iter->second->name.c_str());
	}
	
	const char* pLogDataName[] = {"logData"};
	
	mwSize dims[2] = {1, 1};
	plhs[0] = mxCreateStructArray(2, dims, (int)msgNames.size(), msgNames.data());
	
	const char* pMemberNames[] = {"names", "units", "descs", "data"};
	
	mwSize index = 0;
	for(map<uint16_t, shared_ptr<MessageContainer>>::iterator iter = logData.begin();
		iter != logData.end(); iter++, index++)
	{
		mwSize memberDims[2] = {1, 1};
		mxArray* pMember = mxCreateStructArray(2, memberDims, 4, pMemberNames);
		
		mxSetFieldByNumber(plhs[0], 0, index, pMember);
		
		mwSize nameDims[2] = {1, (mwSize)iter->second->names.size()};
		mxArray* pNames = mxCreateCellArray(2, nameDims);
		mxArray* pUnits = mxCreateCellArray(2, nameDims);
		mxArray* pDescs = mxCreateCellArray(2, nameDims);
		
		mxSetFieldByNumber(pMember, 0, 0, pNames);
		mxSetFieldByNumber(pMember, 0, 1, pUnits);
		mxSetFieldByNumber(pMember, 0, 2, pDescs);
		
		for(size_t j = 0; j < iter->second->names.size(); j++)
		{
			mxSetFieldByNumber(pNames, 0, j, mxCreateString(iter->second->names[j].c_str()));
		}
		
		for(size_t j = 0; j < iter->second->units.size(); j++)
		{
			mxSetFieldByNumber(pUnits, 0, j, mxCreateString(iter->second->units[j].c_str()));
		}

		for(size_t j = 0; j < iter->second->descs.size(); j++)
		{
			mxSetFieldByNumber(pDescs, 0, j, mxCreateString(iter->second->descs[j].c_str()));
		}
		
		mxArray* pData = mxCreateDoubleMatrix(iter->second->data.size(), iter->second->names.size(), mxREAL);
		
		mxSetFieldByNumber(pMember, 0, 3, pData);
		
		mwIndex pos[2];
		
		mwIndex row = 0;
		for(list<vector<double>>::iterator rowIter = iter->second->data.begin();
			rowIter != iter->second->data.end(); rowIter++, row++)
		{
			pos[0] = row;
			
			for(mwIndex col = 0; col < rowIter->size(); col++)
			{
				pos[1] = col;
				
				mxGetPr(pData)[mxCalcSingleSubscript(pData, 2, pos)] = rowIter->at(col);
			}
		}
	}
}
