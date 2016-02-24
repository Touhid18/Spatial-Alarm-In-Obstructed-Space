#include "SAOSServer.h"

SAOSServer::SAOSServer()
{
}


SAOSServer::~SAOSServer()
{
}

SortedLinList * SAOSServer::getAlarmables(GeoLocation q, float radius)
{
	//*****Create an RTree******

	int blocksize = 2048;			//4096;//1024;//2048;
	int b_length = 2048;
	int dimension = 2;

	Cache *cache = new Cache(0, blocksize);
	RTree *rt = new RTree(TREEFILE, cache);

	SortedLinList * result1 = new SortedLinList();

	// TODO Extract q to a float[] center
	//rt->CircleQuery(center, radius, result1);
	result1->print();
	return result1;
}

vector<Obstacle*> SAOSServer::getAllPOIs(float *center, float radius) {
	//*****Create an RTree******
	Cache *cache = new Cache(0, blocksize);
	RTree *rtPOI = new RTree(TREEFILE, cache);

	SortedLinList * resultPOI = new SortedLinList();

	// center={longitude, lattitude}
	/*rtPOI->IO_ACCESS_FILE = new char[13];
	strcpy(rtPOI->IO_ACCESS_FILE, "IOAccessPOI_C");*/
	rtPOI->CircleQuery(center, radius, resultPOI);

	char* IO_ACCESS_FILE = "IOAccessPOI.txt";
	FILE* fp = fopen(IO_ACCESS_FILE, "w+");
	if (fp != NULL && fp != nullptr) {
		int ioAccess = 0;
		fscanf(fp, "%d", &ioAccess);
		if (ioAccess < 1) ioAccess = 0;
		fprintf(fp, "%d", (rtPOI->io_access + ioAccess));
		fclose(fp);
	}

	// resultPOI->print();
	resultPOI->writePOIsInFile();
	//return resultPOI;
	// resultPOI->print();

	vector <Obstacle*> vectPOI;
	string line;
	ifstream visPolyFile(VIS_POLYGON_TXT_FILE_POI);
	if (visPolyFile.is_open())
	{
		while (getline(visPolyFile, line))
		{
			Obstacle *temp = createObstacle(line);// new Obstacle(line);
			vectPOI.push_back(temp);
		}
		visPolyFile.close();
	}

	return vectPOI;
}

vector<Obstacle*>  SAOSServer::getAllObstacles(float * center, float radius)
{
	//******Obstacles RTree**********
	Cache *cacheObst = new Cache(0, blocksize);
	RTree *rtObst = new RTree(TREEFILE_MBR, cacheObst);

	SortedLinList * resultObst = new SortedLinList();

	// center={longitude, lattitude}
	/*rtObst->IO_ACCESS_FILE = new char[13];
	strcpy(rtObst->IO_ACCESS_FILE, "IOAccessObs_C");*/
	rtObst->CircleQuery(center, radius, resultObst);

	char* IO_ACCESS_FILE = "IOAccessObs.txt";
	FILE* fp = fopen(IO_ACCESS_FILE, "w+");
	if (fp != NULL && fp != nullptr) {
		int ioAccess = 0;
		fscanf(fp, "%d", &ioAccess);
		if (ioAccess < 1) ioAccess = 0;
		fprintf(fp, "%d", (rtObst->io_access + ioAccess));
		fclose(fp);
	}

	//resultObst->print();
	resultObst->writeObstaclesInFile();
	// return resultObst;

	vector <Obstacle*> vectObst;
	string line;
	ifstream visPolyFile(VIS_POLYGON_TXT_FILE_Obst);
	if (visPolyFile.is_open())
	{
		while (getline(visPolyFile, line))
		{
			Obstacle *temp = createObstacle(line);// new Obstacle(line);
			vectObst.push_back(temp);
		}
		visPolyFile.close();
	}

	return vectObst;
}

AnswerSet *SAOSServer::getAlarmables(float *q, float range, AnswerSet aPrev)
{
	/*vector<Obstacle*> vectObst = getAllObstacles(q, radius);

	// TODO Combine the POI & Obstacle list => DONE
	for (Obstacle* poi : vectPOI)
		vectObst.push_back(poi);

	// TODO make the visibility graph => DONE
	VisibilityGraph* visGraph = new VisibilityGraph(vectObst);
	VisibilityGraphController* vg = new VisibilityGraphController(visGraph);
	vector<Line*>  visEdges = vg->constructVisGraph();
	vector<Line*> obsSide = visGraph->obsSides;*/
	//visGraph->print();
	vector<Obstacle*> vectPOI = getAllPOIs(q, range);
	int numOfQueryPoints = vectPOI.size();

	// TO_DO Build queryPoints array with all POIs within the range
	/*Point2D* queryPoints = new Point2D[numOfQueryPoints];
	for (int i = 0; i < numOfQueryPoints; i++) {
		/*if (vectPOI[i]->vertices[0]->x < 0
			|| vectPOI[i]->vertices[0]->y < 0) // NO LUCK >:(
			continue; // How the heck a -ve point has come upto this point? >:(
		// => Probably data normalization error :\ - wait ... I didn't normalize it, rather using 09 apus' normalized data-set ... hell yeah - must be grateful to them -_- 
		* /
		queryPoints[i][0] = vectPOI[i]->vertices[0]->x;
		queryPoints[i][1] = vectPOI[i]->vertices[0]->y;

		// if (i > 9)break; // TODO Remove this limitter
	}
	vector<Obstacle*> vectObst = getAllObstacles(q, range);
	int sz = vectPOI.size();
	for (int i = 0; i < sz; i++)
		vectObst.push_back(vectPOI[i]);*/
	//obstructedDistance->writeQueryPointsInFile(queryPoints, numOfQueryPoints);

	ObstructedDistance* obstructedDistance = new ObstructedDistance();
	if (aPrev.visGraph == NULL) {
		// VisibilityGraph* visGraph = 
		aPrev.visGraph = new VisibilityGraph();
		obstructedDistance->constructInitialVisGraph(aPrev.visGraph, vectPOI); // vectObst);
	}

	//******Obstacles RTree**********
	Cache *cacheObst = new Cache(0, blocksize);
	RTree *rtObst = new RTree(TREEFILE_MBR, cacheObst);

	vector<MyStruct> distO_vect = obstructedDistance->getObstructedDistances(aPrev.visGraph,
		q, vectPOI, rtObst);

	AnswerSet* ansSet;
	/*if (AppConstants::ALGO_MODE == AppConstants::MODE_BANDWIDTH_SAVING)
		ansSet = new AnswerSet(GeoLocation(q[0], q[1]), 0, distO_vect);
	else*/
	ansSet = new AnswerSet(GeoLocation(q[0], q[1]), vectPOI, aPrev.visGraph, distO_vect); // vectObst, 

	FILE* fp = fopen("nPOI_Naive.txt", "a");
	fprintf(fp, "%d\n", vectPOI.size());
	fclose(fp);

	return ansSet;
}

float* SAOSServer::getVertex(float * focus, VectorLine sAxis, float a) {
	float u[] = { focus[0] - a *sAxis.dx, focus[1] - a *sAxis.dy };
	return u;
}

bool arePointsOnTheSameSide(float ax, float ay, float bx, float by, float* stLine) {
	// stLine[0,1,2] = (a,b,c)
	float fVal = stLine[0] * ax + stLine[1] * ay + stLine[2];
	float pVal = stLine[0] * bx + stLine[1] * by + stLine[2];
	if ((fVal > 0 && pVal < 0) || (fVal < 0 && pVal > 0) || pVal != 0)
		return false;
	return true;
}

int checkCollision(BoundedParabola obstParabola, vector<Obstacle*> vectObst) {
	int sz = vectObst.size();
	// Parabola's collision: If 1st vertex is inside the parabola, then others must be
	// or, for outside, others must be outside

	// Bounding straight line: All vertices must be at the same side of p.sb
	float fx = obstParabola.focusX, fy = obstParabola.focusY;
	float bl[] = { obstParabola.aBoundLine, obstParabola.bBoundLine, obstParabola.cBoundLine };

	for (int i = 0; i < sz; i++) {
		vector<Point*> verticesOfObst = vectObst[i]->vertices;

		bool insideP = obstParabola.isPointInsideParabola(
			verticesOfObst[0]->x, verticesOfObst[0]->y);
		bool atFocalSide = arePointsOnTheSameSide( fx, fy,
			verticesOfObst[0]->x, verticesOfObst[0]->y, bl);

		int vsz = verticesOfObst.size();
		for (int j = 1; j < vsz; j++) {
			bool isAlsoInP = obstParabola.isPointInsideParabola(
				verticesOfObst[j]->x, verticesOfObst[j]->y);
			if (insideP != isAlsoInP)
				return 1;
			// TODO Check whether have collision with the st. line
			bool atFocalSideP = arePointsOnTheSameSide(fx, fy,
				verticesOfObst[j]->x, verticesOfObst[j]->y, bl);
			if (atFocalSide != atFocalSideP)
				return 2;
		}
	}
	return -1;
}

vector<Obstacle*> SAOSServer::getPoisWithinParabola(BoundedParabola poiParabola) {

	//*****Create an RTree******
	Cache *cache = new Cache(0, blocksize);
	RTree *rtPOI = new RTree(TREEFILE, cache);

	SortedLinList * resultPOI = new SortedLinList();

	// center={longitude, lattitude}
	/*rtPOI->IO_ACCESS_FILE = new char[13];
	strcpy(rtPOI->IO_ACCESS_FILE, "IOAccessPOI_C");*/
	// rtPOI->CircleQuery(center, radius, resultPOI);
	rtPOI->ParabolaQuery(poiParabola, resultPOI);

	char* IO_ACCESS_FILE = "IOAccessPOI_Main.txt";
	FILE* fp = fopen(IO_ACCESS_FILE, "w+");
	if (fp != NULL && fp != nullptr) {
		int ioAccess = 0;
		fscanf(fp, "%d", &ioAccess);
		if (ioAccess < 1) ioAccess = 0;
		fprintf(fp, "%d", (rtPOI->io_access + ioAccess));
		fclose(fp);
	}
	return resultPOI->getVectorOfPOI();

	// resultPOI->print();
	// resultPOI->writePOIsInFile();
	//return resultPOI;
	// resultPOI->print();

	// vector <Obstacle*> vectPOI = 

	/*string line;
	ifstream visPolyFile(VIS_POLYGON_TXT_FILE_POI);
	if (visPolyFile.is_open())
	{
		while (getline(visPolyFile, line))
		{
			Obstacle *temp = createObstacle(line);// new Obstacle(line);
			vectPOI.push_back(temp);
		}
		visPolyFile.close();
	}

	return vectPOI;*/
}

vector<Obstacle*>  SAOSServer::getObsWithinParabola(BoundedParabola obstParabola)
{
	//******Obstacles RTree**********
	Cache *cacheObst = new Cache(0, blocksize);
	RTree *rtObst = new RTree(TREEFILE_MBR, cacheObst);

	SortedLinList * resultObst = new SortedLinList();

	rtObst->ParabolaQuery(obstParabola, resultObst);

	char* IO_ACCESS_FILE = "IOAccessObst_Main.txt";
	FILE* fp = fopen(IO_ACCESS_FILE, "w+");
	if (fp != NULL && fp != nullptr) {
		int ioAccess = 0;
		fscanf(fp, "%d", &ioAccess);
		if (ioAccess < 1) ioAccess = 0;
		fprintf(fp, "%d", (rtObst->io_access + ioAccess));
		fclose(fp);
	}
	return resultObst->getVectorOfObstacles();

	//resultObst->print();
	// resultObst->writeObstaclesInFile();
	/*// return resultObst;

	vector <Obstacle*> vectObst;
	string line;
	ifstream visPolyFile(VIS_POLYGON_TXT_FILE_Obst);
	if (visPolyFile.is_open())
	{
		while (getline(visPolyFile, line))
		{
			Obstacle *temp = createObstacle(line);// new Obstacle(line);
			vectObst.push_back(temp);
		}
		visPolyFile.close();
	}

	return vectObst;*/
}

AnswerSet * SAOSServer::queryAlarmables(float * q, float radius, VectorLine sAxis,
	AnswerSet aPrev, 
	float mp, float np)
{
	AnswerSet* ansSet;
	VisibilityGraph* visGraph = new VisibilityGraph();
	ObstructedDistance* obstructedDistance = new ObstructedDistance();
	vector<MyStruct> distO_vect;

	float* u = getVertex(q, sAxis, mp*radius);

	float mDirtx = -1.0 / sAxis.m;
	float cDirtx = (2 * u[1] - q[1]) - mDirtx * (2 * u[0] - q[0]);

	float mBnd = mDirtx;
	float bx = q[0] + (np*radius) * sAxis.dx;
	float by = q[1] + (np*radius) * sAxis.dy;
	float cBnd = by - mBnd*bx;

	BoundedParabola poiParabola = BoundedParabola(q[0], q[1],
						mDirtx, -1.0, cDirtx, 
						mBnd, -1.0, cBnd);

	vector<Obstacle*> vectPOI = getPoisWithinParabola(poiParabola);
	while (vectPOI.size() < 1 && mp<radius && np<radius) {
		float f[] = {poiParabola.focusX, poiParabola.focusY};
		mp += 2;
		np += 2;
		if (mp > 90)
			printf("\n");
		poiParabola = BoundedParabola(f, poiParabola.sAxis,
			mp*radius, np*radius);
		vectPOI = getPoisWithinParabola(poiParabola);
	}
	if (vectPOI.size() < 1) {
		ansSet = new AnswerSet(GeoLocation(q[0], q[1]), poiParabola, poiParabola, 
			vectPOI, aPrev.visGraph, distO_vect);
		return ansSet;
	}

	vector<Obstacle*> vectObst = getObsWithinParabola(poiParabola);
	float mo = mp;
	float no = np;
	BoundedParabola obstParabola = poiParabola;

	int f = checkCollision(obstParabola, vectObst);
	while (f > 0) {
		if (f == 2)// Collision with the bounding st. line
			no++;
		else // Collision with the obstacles' parabola
			mo++;
		obstParabola = BoundedParabola(q, sAxis, mo*radius, no*radius);
		vector<Obstacle*> vectPoiP = getPoisWithinParabola(obstParabola);
		vector<Obstacle*> vectObsP = getObsWithinParabola(obstParabola);

		if (vectPoiP.size() > vectPOI.size()) {
			vectPOI = vectPoiP;
			mp = mo;
			np = no;
			poiParabola = obstParabola;
		}
		vectObst = vectObsP;
		f = checkCollision(obstParabola, vectObst);
	}/**/

	/*int numOfQueryPoints = vectPOI.size();
	Point2D* queryPoints = new Point2D[numOfQueryPoints];
	for (int i = 0; i < numOfQueryPoints; i++) {
		queryPoints[i][0] = vectPOI[i]->vertices[0]->x;
		queryPoints[i][1] = vectPOI[i]->vertices[0]->y;
	}

	obstructedDistance->writeQueryPointsInFile(queryPoints, numOfQueryPoints);
	vector<string> obstLines;
	for (int i = 0; i < numOfQueryPoints; i++) {
		stringstream ss;
		ss << "polygon((" << queryPoints[i][0] << " " << queryPoints[i][1] << "," 
			<< queryPoints[i][0] << " " << queryPoints[i][1]<< "))\n";
		obstLines.push_back(ss.str());
	}*/
	obstructedDistance->constructInitialVisGraph(visGraph, vectObst);// , obstLines);

	//******Obstacles RTree**********
	Cache *cacheObst = new Cache(0, blocksize);
	RTree *rtObst = new RTree(TREEFILE_MBR, cacheObst);

	distO_vect = obstructedDistance->getObstructedDistances(visGraph, q, vectPOI, rtObst);

	ansSet = new AnswerSet(GeoLocation(q[0], q[1]), poiParabola, poiParabola, //obstParabola,
		vectPOI, aPrev.visGraph, distO_vect); // vectObst, 

	FILE* fp = fopen("nPOI_Main.txt", "a");
	fprintf(fp, "%d %d\n", vectPOI.size(), sizeof(visGraph));
	fclose(fp);

	return ansSet;
}

AnswerSet* SAOSServer::getKnownRegionData(float * q, float radius, VectorLine sAxis,
	AnswerSet aPrev, 
	float mp, float np) {

	float* u = getVertex(q, sAxis, mp*radius);
	float mDirtx = -1.0 / sAxis.m;
	float cDirtx = (2 * u[1] - q[1]) - mDirtx * (2 * u[0] - q[0]);
	float mBnd = mDirtx;
	float bx = q[0] + (np*radius) * sAxis.dx;
	float by = q[1] + (np*radius) * sAxis.dy;
	float cBnd = by - mBnd*bx;
	BoundedParabola poiParabola = BoundedParabola(q[0], q[1],
		mDirtx, -1.0, cDirtx,
		mBnd, -1.0, cBnd);
	vector<Obstacle*> vectPOI = getPoisWithinParabola(poiParabola);

	// VisibilityGraph* visGraph = new VisibilityGraph();
	ObstructedDistance* obstructedDistance = new ObstructedDistance();
	// vector<Obstacle*> vectObst = getObsWithinParabola(poiParabola);
	if(aPrev.visGraph==NULL || aPrev.visGraph==nullptr)
		obstructedDistance->constructInitialVisGraph(aPrev.visGraph, vectPOI);

	//******Obstacles RTree**********
	Cache *cacheObst = new Cache(0, blocksize);
	RTree *rtObst = new RTree(TREEFILE_MBR, cacheObst);

	vector<MyStruct> distO_vect = obstructedDistance->getObstructedDistances(aPrev.visGraph, q,
		vectPOI, rtObst);

	AnswerSet* ansSet;
	/*if (AppConstants::ALGO_MODE == AppConstants::MODE_BANDWIDTH_SAVING)
	ansSet = new AnswerSet(GeoLocation(q[0], q[1]), 0, distO_vect);
	else*/
	ansSet = new AnswerSet(GeoLocation(q[0], q[1]), vectPOI, aPrev.visGraph, distO_vect); // vectObst, 

	FILE* fp = fopen("nPOI_Main.txt", "a");
	fprintf(fp, "%d %d\n", sizeof(vectPOI), sizeof(distO_vect));
	fclose(fp);

	return ansSet;
}