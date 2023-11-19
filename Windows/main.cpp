#include <iostream>
#include <stdlib.h>
#include "BitmapRawConverter.h"
#include "tbb/blocked_range.h"
#include "tbb/parallel_for.h"
#include "tbb/tick_count.h"

#define __ARG_NUM__				6
#define FILTER_SIZE3			3
#define FILTER_SIZE5			5
#define THRESHOLD				128

using namespace std;
using namespace tbb;

// Prewitt operators
int filterHor5[FILTER_SIZE5 * FILTER_SIZE5] = {9, 9, 9, 9, 9, 9, 5, 5, 5, 9, -7, -3, 0, -3, -7, -7, -3, -3, -3, -7, -7, -7, -7, -7, -7};
int filterVer5[FILTER_SIZE5 * FILTER_SIZE5] = {9, 9, -7, -7, -7, 9, 5, -3, -3, -7, 9 ,5, 0 ,-3, -7, 9, 5, -3, -3, -7, 9, 9, -7, -7, -7};
int filterHor3[FILTER_SIZE3 * FILTER_SIZE3] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };
int filterVer3[FILTER_SIZE3 * FILTER_SIZE3] = { -1, -1, -1, 0, 0, 0, 1, 1, 1 };

/**
* @brief Serial version of edge detection algorithm implementation using Prewitt operator
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param width image width
* @param height image height
* @param filterHor horizontal Prewitt filter
* @param filterVer vertical Prewitt filter
* @param filterSize size of Prewitt filter
*/
void filter_serial_prewitt(int *inBuffer, int *outBuffer, int width, int height, int *filterHor, int *filterVer, int filterSize)
{
	int margin = filterSize;
	for (int i = margin; i < width - margin; i++) {
		for (int j = margin; j < height - margin; j++) {
			int Gx = 0;
			int Gy = 0;
			int G = 0;
			for (int m = 0; m < filterSize; m++) {
				for (int n = 0; n < filterSize; n++) {
					Gx += inBuffer[(j + n - 1) * width + (i + m - 1)] * filterHor[n * filterSize + m];
					Gy += inBuffer[(j + n - 1) * width + (i + m - 1)] * filterVer[n * filterSize + m];
				}
			}
			
			G = abs(Gx) + abs(Gy);
			if (G > THRESHOLD) {
				outBuffer[j * width + i] = 255;
			}
			else {
				outBuffer[j * width + i] = 0;
			}
		}
	}
}

/**
* @brief Parallel version of edge detection algorithm implementation using Prewitt operator
* 
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param width current image width
* @param height current image height
* @param row current row of input image
* @param col current col of input image
* @param fullWidth full width of input image
* @param fullHeight full height of input image
* @param filterHor horizontal Prewitt filter
* @param filterVer vertical Prewitt filter
* @param filterSize size of Prewitt filters
*/
void filter_parallel_prewitt(int *inBuffer, int *outBuffer, int width, int height, int row, int col, int fullWidth, int fullHeight, int* filterHor, int* filterVer, int filterSize)
{
	task_group g;

	if (width <= fullWidth/10) {
		int margin = filterSize;
		int rowFirst = row;
		int rowLast = row + width;
		int colFirst = col;
		int colLast = col + height;
		if (row == 0) {
			rowFirst = margin;
		}if (col == 0) {
			colFirst = margin;
		}if (rowLast == fullWidth) {
			rowLast = row + width - margin;
		}if (colLast == fullHeight) {
			colLast = col + height - margin;
		}
		

		for (int i = rowFirst; i < rowLast; i++) {
			for (int j = colFirst; j < colLast; j++) {
				int Gx = 0;
				int Gy = 0;
				int G = 0;
				for (int m = 0; m < filterSize; m++) {
					for (int n = 0; n < filterSize; n++) {
						Gx += inBuffer[(j + n - 1) * fullWidth + (i + m - 1)] * filterVer[n * filterSize + m];
						Gy += inBuffer[(j + n - 1) * fullWidth + (i + m - 1)] * filterHor[n * filterSize + m];
					}
				}
				G = abs(Gx) + abs(Gy);
				if (G > THRESHOLD) {
					outBuffer[j * fullWidth + i] = 255;
				}
				else {
					outBuffer[j * fullWidth + i] = 0;
				}
			}
		}
	}
	else {
		int widthMiddle = width / 2;
		int heightMiddle = height / 2;
		g.run([&] {filter_parallel_prewitt(inBuffer, outBuffer, widthMiddle, heightMiddle, row, col, fullWidth, fullHeight, filterHor, filterVer, filterSize);});
		g.run([&] {filter_parallel_prewitt(inBuffer, outBuffer, widthMiddle, heightMiddle, row + widthMiddle, col, fullWidth, fullHeight, filterHor, filterVer, filterSize);});
		g.run([&] {filter_parallel_prewitt(inBuffer, outBuffer, widthMiddle, heightMiddle, row, col + heightMiddle, fullWidth, fullHeight, filterHor, filterVer, filterSize);});
		g.run([&] {filter_parallel_prewitt(inBuffer, outBuffer, widthMiddle, heightMiddle, row + widthMiddle, col + heightMiddle, fullWidth, fullHeight, filterHor, filterVer, filterSize);});
		g.wait();
	}
	
}


/**
* @brief Serial version of edge detection algorithm
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param width image width
* @param height image height
* @param surrounding size of matrix for pixel surrounding
*/
void filter_serial_edge_detection(int *inBuffer, int *outBuffer, int width, int height, int surrounding)
{
	for (int i = surrounding; i < width - surrounding; i++) {
		for (int j = surrounding; j < height - surrounding; j++) {
			int P = 0;
			int O = 1;
			int F = 0;
			for (int m = 0; m < surrounding; m++) {
				for (int n = 0; n < surrounding; n++) {
					if (inBuffer[(j + n - 1) * width + (i + m - 1)] > THRESHOLD) {
						P = 1;
					}
					if (inBuffer[(j + n - 1) * width + (i + m - 1)] < THRESHOLD) {
						O = 0;
					}
				}
			}
			F = abs(P - O);
			if (F == 1) {
				outBuffer[j * width + i] = 255;
			}
			else {
				outBuffer[j * width + i] = 0;
			}
		}
	}
}

/**
* @brief Parallel version of edge detection algorithm
* 
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param width image width
* @param height image height
* @param row current row of input image
* @param col current col of input image
* @param fullWidth full width of input image
* @param fullHeight full height of input image
* @param surrounding size of matrix for pixel surrounding
*/
void filter_parallel_edge_detection(int *inBuffer, int *outBuffer, int width, int height, int row, int col, int fullWidth, int fullHeight, int surrounding)
{
	task_group g;

	if (width <= fullWidth / 10) {
		int rowFirst = row;
		int rowLast = row + width;
		int colFirst = col;
		int colLast = col + height;
		if (row == 0) {
			rowFirst = surrounding;
		}if (col == 0) {
			colFirst = surrounding;
		}if (rowLast == fullWidth) {
			rowLast = row + width - surrounding;
		}if (colLast == fullHeight) {
			colLast = col + height - surrounding;
		}


		for (int i = rowFirst; i < rowLast; i++) {
			for (int j = colFirst; j < colLast; j++) {
				int P = 0;
				int O = 1;
				int F = 0;
				for (int m = 0; m < surrounding; m++) {
					for (int n = 0; n < surrounding; n++) {
						if (inBuffer[(j + n - 1) * fullWidth + (i + m - 1)] > THRESHOLD) {
							P = 1;
						}
						if (inBuffer[(j + n - 1) * fullWidth + (i + m - 1)] < THRESHOLD) {
							O = 0;
						}
					}
				}
				F = abs(P - O);
				if (F == 1) {
					outBuffer[j * fullWidth + i] = 255;
				}
				else {
					outBuffer[j * fullWidth + i] = 0;
				}
			}
		}
	}
	else {
		int widthMiddle = width / 2;
		int heightMiddle = height / 2;
		g.run([&] {filter_parallel_edge_detection(inBuffer, outBuffer, widthMiddle, heightMiddle, row, col, fullWidth, fullHeight, surrounding);});
		g.run([&] {filter_parallel_edge_detection(inBuffer, outBuffer, widthMiddle, heightMiddle, row + widthMiddle, col, fullWidth, fullHeight, surrounding);});
		g.run([&] {filter_parallel_edge_detection(inBuffer, outBuffer, widthMiddle, heightMiddle, row, col + heightMiddle, fullWidth, fullHeight, surrounding);});
		g.run([&] {filter_parallel_edge_detection(inBuffer, outBuffer, widthMiddle, heightMiddle, row + widthMiddle, col + heightMiddle, fullWidth, fullHeight, surrounding);});
		g.wait();
	}
}

/**
* @brief Function for running test.
*
* @param testNr test identification, 1: for serial version, 2: for parallel version
* @param ioFile input/output file, firstly it's holding buffer from input image and than to hold filtered data
* @param outFileName output file name
* @param outBuffer buffer of output image
* @param width image width
* @param height image height
* @param prewitt user choice for dimension of Prewitt filter
* @param edge user choice for dimension of pixel surroungings
*/
void run_test_nr(int testNr, BitmapRawConverter* ioFile, char* outFileName, int* outBuffer, unsigned int width, unsigned int height, string prewitt, int edge)
{
	tick_count startTime;
	tick_count endTime;

	switch (testNr)
	{
		case 1:
			cout << "\n\n=========================================================================" << endl;
			cout << "Running serial version of edge detection using Prewitt operator" << endl;
			startTime = tick_count::now();
			if (prewitt == "1") {
				filter_serial_prewitt(ioFile->getBuffer(), outBuffer, width, height, filterHor3, filterVer3, 3);
			}
			else if (prewitt == "2") {
				
				filter_serial_prewitt(ioFile->getBuffer(), outBuffer, width, height, filterHor5, filterVer5, 5);
			}
			endTime = tick_count::now();
			cout << "Serial Prewitt took " << (endTime - startTime).seconds() << " seconds" << endl;
			break;
		case 2:
			cout << "\n\n=========================================================================" << endl;
			cout << "Running parallel version of edge detection using Prewitt operator" << endl;

			startTime = tick_count::now();
			if (prewitt == "1") {
				filter_parallel_prewitt(ioFile->getBuffer(), outBuffer, width, height, 0, 0, width, height, filterHor3, filterVer3, 3);
			}
			else if (prewitt == "2") {

				filter_parallel_prewitt(ioFile->getBuffer(), outBuffer, width, height, 0, 0, width, height, filterHor5, filterVer5, 5);
			}
			endTime = tick_count::now();

			cout << "Parallel Prewitt took " << (endTime - startTime).seconds() << " seconds" << endl;
			break;
		case 3:
			cout << "\n\n=========================================================================" << endl;
			cout << "Running serial version of edge detection" << endl;
			startTime = tick_count::now();
			filter_serial_edge_detection(ioFile->getBuffer(), outBuffer, width, height, edge);
			endTime = tick_count::now();
			cout << "Serial edge detection took " << (endTime - startTime).seconds() << " seconds" << endl;
			break;
		case 4:
			cout << "\n\n=========================================================================" << endl;
			cout << "Running parallel version of edge detection" << endl;
			startTime = tick_count::now();
			filter_parallel_edge_detection(ioFile->getBuffer(), outBuffer, width, height, 0, 0, width, height, edge);
			endTime = tick_count::now();
			cout << "Parallel edge detection took " << (endTime - startTime).seconds() << " seconds" << endl;
			cout << "=========================================================================\n\n" << endl;
			break;
		default:
			cout << "ERROR: invalid test case, must be 1, 2, 3 or 4!";
			break;
	}

	ioFile->setBuffer(outBuffer);
	ioFile->pixelsToBitmap(outFileName);
}

/**
* @brief Print program usage.
*/
void usage()
{
	cout << "\n\ERROR: call program like: " << endl << endl; 
	cout << "ProjekatPP.exe";
	cout << " input.bmp";
	cout << " outputSerialPrewitt.bmp";
	cout << " outputParallelPrewitt.bmp";
	cout << " outputSerialEdge.bmp";
	cout << " outputParallelEdge.bmp" << endl << endl;
}

int main(int argc, char * argv[])
{

	if(argc != __ARG_NUM__)
	{
		usage();
		return 0;
	}

	BitmapRawConverter inputFile(argv[1]);
	BitmapRawConverter outputFileSerialPrewitt(argv[1]);
	BitmapRawConverter outputFileParallelPrewitt(argv[1]);
	BitmapRawConverter outputFileSerialEdge(argv[1]);
	BitmapRawConverter outputFileParallelEdge(argv[1]);

	unsigned int width, height;

	int test;
	
	width = inputFile.getWidth();
	height = inputFile.getHeight();

	int* outBufferSerialPrewitt = new int[width * height];
	int* outBufferParallelPrewitt = new int[width * height];

	memset(outBufferSerialPrewitt, 0x0, width * height * sizeof(int));
	memset(outBufferParallelPrewitt, 0x0, width * height * sizeof(int));

	int* outBufferSerialEdge = new int[width * height];
	int* outBufferParallelEdge = new int[width * height];

	memset(outBufferSerialEdge, 0x0, width * height * sizeof(int));
	memset(outBufferParallelEdge, 0x0, width * height * sizeof(int));

	string prewitt;
	int edge;

	cout << "What Prewitt operator dimension do you want? " << "\n1. 3x3" << "\n2. 5x5" << "\n>>  ";
	cin >> prewitt;

	cout << "Enter surrounding width for edge detection" << "\n>>  ";
	cin >> edge;
	edge = 2 * edge + 1;

	// serial version Prewitt
	run_test_nr(1, &outputFileSerialPrewitt, argv[2], outBufferSerialPrewitt, width, height, prewitt, edge);

	// parallel version Prewitt
	run_test_nr(2, &outputFileParallelPrewitt, argv[3], outBufferParallelPrewitt, width, height, prewitt, edge);

	// serial version special
	run_test_nr(3, &outputFileSerialEdge, argv[4], outBufferSerialEdge, width, height, prewitt, edge);

	// parallel version special
	run_test_nr(4, &outputFileParallelEdge, argv[5], outBufferParallelEdge, width, height, prewitt, edge);

	// verification
	cout << "Verification: ";
	test = memcmp(outBufferSerialPrewitt, outBufferParallelPrewitt, width * height * sizeof(int));

	if(test != 0)
	{
		cout << "Prewitt FAIL!" << endl;
	}
	else
	{
		cout << "Prewitt PASS." << endl;
	}

	test = memcmp(outBufferSerialEdge, outBufferParallelEdge, width * height * sizeof(int));

	if(test != 0)
	{
		cout << "Edge detection FAIL!" << endl;
	}
	else
	{
		cout << "Edge detection PASS." << endl;
	}

	//clean up
	delete outBufferSerialPrewitt;
	delete outBufferParallelPrewitt;

	delete outBufferSerialEdge;
	delete outBufferParallelEdge;

	return 0;
} 