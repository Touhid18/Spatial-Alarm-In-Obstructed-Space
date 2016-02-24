#include "Client.h"
#include <chrono>
#include <functional>
#include <thread>
#include <iostream>
#include <math.h>
#include "visGraph/obstacles.h"
#include "visGraph/dijkstra.h"
#include "BoundedParabola.h"
using namespace std;

class Stopwatch {
	time_t t1, t2;
	clock_t c1, c2;
public:
	void start() {
		c1 = clock();
		t1 = time(0);
	}
	;
	void stop() {
		c2 = clock();
		t2 = time(0);
	}
	;
	int getDiff() {
		return (c2 - c1);
	}
	;
};

// Total 14 obstacles within a 26x13 (XxY) grid
//Obstacle DUMMY_OBSTACLES[] = { Obstacle(1,1,5,2), Obstacle(0,1,2,6),
//							Obstacle(1,7,8,8), Obstacle(0,9,4,13),
//							Obstacle(1,11,13,12), Obstacle(13,2,20,3),
//							Obstacle(10,6,11,10), Obstacle(13,6,14,10),
//							Obstacle(16,6,17,10), Obstacle(19,6,20,10),
//							Obstacle(22,6,23,10), Obstacle(25,6,26,10),
//							Obstacle(8,3,16,4), Obstacle(17,3,26,4) };
const int ALARM_RADIUS = 100; // according to the normalized X-Y Space
const char *CLIENT_INFO_FILE = "ClientInfo.txt";
const int X_UP_LIMIT = 8520;
const int Y_UP_LIMIT = 8520;

AnswerSet *curAnsSet;
Stopwatch timeSpan;
long double totalSession = 0;

BoundedParabola relParabola;
float rSafe;

long double algoRunTimes[11];
long int algoOccurs[11];
char* outFileArr[] = { "algo1.thd", "algo2.thd", "algo3.thd", "algo4.thd", "algo5.thd", "algo6.thd" };
const int N_ALGO = 6;

float lastX, lastY;

vector<pair<float, float>> readClientFromInfoFile(int clientId) {
	FILE* fp = fopen(CLIENT_INFO_FILE, "r");

	float clientIdCur = 0, curM, curC, curVelocity, curX, curY;

	if (fp == NULL || fp == nullptr) {
		printf("\n\tERROR : ClientInfoFile.txt file not found!\n");
		return vector<pair<float, float>>();
	}
	fscanf(fp, "%f %f %f %f %f %f", &clientIdCur, &curM, &curC, &curVelocity, &curX, &curY);

	vector<pair<float, float>> retVect;
	retVect.push_back(make_pair(clientIdCur, curM));
	retVect.push_back(make_pair(curC, curVelocity));
	retVect.push_back(make_pair(curX, curY));

	float m, c;
	while (fscanf(fp, "%f %f", &m, &c) == 2)
		retVect.push_back(make_pair(m, c));
	fclose(fp);

	//float ret[] = { clientIdCur, curM, curC, curVelocity, curX, curY };
	if (clientId == clientIdCur) {
		printf("valid client info. read as: id=%d, m=%.2f, c=%.2f, vel.=%.2f, longi.=%.2f, lati.=%.2f\n", clientId, curM, curC, curVelocity, curX, curY);
		return retVect;
	}
	else {
		printf("ERROR : Client ID mismatched!\n");
		return vector<pair<float, float>>();
	}/**/
}

void writeClientInfoInFile(int clientId, float curM, float curC, float curVelocity,
	float curX, float curY, vector<pair<float, float>> pathHistoryVect) {

	int histLen = pathHistoryVect.size();

	FILE* fp = fopen(CLIENT_INFO_FILE, "w");
	if (fp != NULL && fp != nullptr) {
		fprintf(fp, "%d %f %f %f %f %f",
			clientId, curM, curC, curVelocity, curX, curY);

		int i = histLen > 10 ? 1 : 0; // Skipping the oldest (first, i=0) entry to constraint history_limit = 10
		for (; i < histLen; i++)
			fprintf(fp, "\n%f %f", pathHistoryVect[i].first, pathHistoryVect[i].second);

		fclose(fp);
	}
}

void writePerformanceParams() {
	for (int i = 0; i < N_ALGO; i++) {
		FILE* outFp = fopen(outFileArr[i], "w");
		fprintf(outFp, "%d %f", algoOccurs[i + 1], algoRunTimes[i + 1]);
		fclose(outFp);
	}
}

Client::~Client()
{
}

Client::Client(int clientId)
{
	printf("\nClient created : id=%d\n", clientId);
	this->clientId = clientId;
	this->clientLocation = GeoLocation();
	this->curC = 5;
	this->curM = 1;
	this->curVelocity = 20.5; // 20.5 normalized units/hour
	this->pathHistoryVect.push_back(make_pair(this->curM, this->curC));
	activateClient();
}

Client::Client(int clientId, GeoLocation curClientLoc)
{
	printf("\nClient created : id=%d, location = \n", clientId, curClientLoc);
	this->clientId = clientId;
	this->clientLocation = GeoLocation(curClientLoc);
	this->curC = 5;
	this->curM = 1;
	this->curVelocity = 20.5; // 20.5 normalized units/hour
	this->pathHistoryVect.push_back(make_pair(this->curM, this->curC));
	activateClient();
}

Client::Client(int clientId, float curM, float curC, float curVelocity,
	float curX, float curY)
{
	// printf("\nClient created : id=%d, location = (%d, %d)\n", clientId, curX, curY);
	this->clientId = clientId;
	this->clientLocation = GeoLocation(curX, curY);
	this->curC = curC;
	this->curM = curM;
	this->curVelocity = curVelocity; // normalized units/hour
	this->pathHistoryVect.push_back(make_pair(this->curM, this->curC));
	activateClient();
}

// Utility methods


double obstructedDistance(VisibilityGraph* vg, double sourceX, double sourceY,
	double destX, double destY) {
	int numOfPoints = vg->nodes.size();
	int numOfEdges = vg->edges.size();
	int sourcePointId = searchPointByCoord(vg->nodes, sourceX, sourceY)->id;
	int destPointId = searchPointByCoord(vg->nodes, destX, destY)->id;
	printf("\nFinding shortest path from %d -> %d\n", sourcePointId,
		destPointId, numOfEdges, numOfPoints);
	Point* start;
	Point* goal;
	double dist = initiateDijkstra(numOfPoints, numOfEdges, false, sourcePointId,
		destPointId, 1000);
	vector<int> shortestPath = getShortestPath();
	int i = 0;
	return dist;
	//Print the Shortest Path
	/*printf("The Shortest Path is :");
	while (shortestPath[i] != -1) {
	printf("%d ", shortestPath[i]);

	if (shortestPath[i + 1] != -1) {
	start = getPointById(vg->nodes, shortestPath[i]);
	goal = getPointById(vg->nodes, shortestPath[i + 1]);
	// Visualize:
	/*drawCircle(start->x, start->y, 2, GREEN);
	drawCircle(goal->x, goal->y, 2, GREEN);
	drawLine(start->x, start->y, goal->x, goal->y, GREEN);* /
	}
	else {
	goal = getPointById(vg->nodes, destPointId);
	// writeText((goal->x - 5), (goal->y + 15), "Dest", GREEN);
	}

	if (i == 0) {
	// writeText((start->x - 5), (start->y + 15), "Source", GREEN);
	}

	i++;
	}*/
}

double euclideanDist(double sourceX, double sourceY, double destX, double destY) {
	double dx = sourceX - destX;
	double dy = sourceY - destY;

	return sqrt(dx*dx + dy*dy);
}

void doAlarmUser(Obstacle *poi) {
	printf("\n\t*** ALARM :: POI Reached: longitude=%f, latitude=%f\n",
		poi->vertices[0]->x, poi->vertices[0]->y);
}
void doAlarmUser(float* poiXY) {
	printf("\n\t*** ALARM :: POI Reached: longitude=%f, latitude=%f\n",
		poiXY[0], poiXY[1]);
}

AnswerSet * getUnifiedAnswerSet(AnswerSet newAnsSet) {
	AnswerSet *as = new AnswerSet();
	as->location = newAnsSet.location;
	as->minDistToUpdate = newAnsSet.minDistToUpdate;
	as->obsParabola = newAnsSet.obsParabola;
	as->poiParabola = newAnsSet.poiParabola;

	if (curAnsSet == NULL || curAnsSet == nullptr) {
		as->poiSet = newAnsSet.poiSet;
		as->distO_vect = newAnsSet.distO_vect;
		return as;
	}
	// as->visGraph = newAnsSet.location;

	int m = curAnsSet->distO_vect.size();
	int n = newAnsSet.distO_vect.size();
	for (int i = 0; i < n; i++) {
		bool isNewPoi = true;
		for (int j = 0; j < m; j++) {
			if ((curAnsSet->distO_vect[j].queryPoints[0]
				== newAnsSet.distO_vect[i].queryPoints[0])
				&& (curAnsSet->distO_vect[j].queryPoints[1]
				== newAnsSet.distO_vect[i].queryPoints[1])) {
				isNewPoi = false;
				break;
			}
		}
		if (isNewPoi)
			as->distO_vect.push_back(newAnsSet.distO_vect[i]);
	}
	n = curAnsSet->poiSet.size();
	m = newAnsSet.poiSet.size();
	for (int i = 0; i < m; i++) {
		bool isNewPoi = true;
		for (int j = 0; j < n; j++) {
			if (curAnsSet->poiSet[j]->id
				== newAnsSet.poiSet[i]->id) {
				isNewPoi = false;
				break;
			}
		}
		if (isNewPoi)
			as->poiSet.push_back(newAnsSet.poiSet[i]);
	}
	return as;
}

// Algo 5
float ConfigUpdate(float posX, float posY) {

	float rSafe = 10000000.0;
	for (Obstacle* poi : curAnsSet->poiSet) {

		int nPois = curAnsSet->distO_vect.size();
		for (int i = 0; i < nPois; i++) {
			if (curAnsSet->distO_vect[i].distance < ALARM_RADIUS)
				doAlarmUser(curAnsSet->distO_vect[i].queryPoints);
			else { /// TODO Decide alarm again for the same POI ??
				float ed = euclideanDist(posX, posY,
					curAnsSet->distO_vect[i].queryPoints[0],
					curAnsSet->distO_vect[i].queryPoints[1]);
				if (ed < rSafe)
					rSafe = ed;
			}
		}
	}
	// Write parabola(focus, directrix & bounding line) & rSafe to file
	// FILE *fp = fopen("CurRelSafeRegions.txt", "w");
	// if(fp!=NULL || fp!=nullptr)
	// fprintf(fp, "%f %f %f %f %f %f %f %f\n%f", pi);
	// BoundedParabola piR(piP, ALARM_RADIUS);

	return (rSafe -= ALARM_RADIUS);
}

VectorLine predictDirection(float x, float y, vector<pair<float, float>> pathHistory) {
	float m = 0, c = 0, sumM = 0, w = 0.1, totalW = 0.0, dx, dy;
	int dir = +1;

	int sz = pathHistory.size();
	if (sz < 1)
		return VectorLine(1.0, 5.0, 0.7071, 0.7071); // y=x+5 in (i+j)/(root(2)) direction
	else if (sz < 2) {
		m = pathHistory[0].first;
		c = pathHistory[0].second;
		dx = 1;
		dy = m*x + c;
		float dxy = sqrt(dx*dx + dy*dy);
		dx /= dxy;
		dy /= dxy;
		return  VectorLine(m, c, dx, dy);
	}

	// Weighted sum for the slope
	for (int i = 0; i < sz; i++) {
		float ss = pathHistory[i].first * w;
		if (ss < 0) ss *= (-1);
		sumM += ss;
		totalW += w;
		w += 0.1;
	}
	m = sumM / totalW;
	c = y - m*x;

	// size must be at least 2
	// If the latest two lines' directions are parallell/same, then assign the latest direction as the next direction
	if (pathHistory[sz - 2].first == pathHistory[sz - 1].first)
		dx = lastX;
	else
		dx = ((pathHistory[sz - 2].second - pathHistory[sz - 1].second)
			/ (pathHistory[sz - 2].first - pathHistory[sz - 1].first)) - 2 * x;
	dy = m*dx + c;

	float dxy = sqrt(dx*dx + dy*dy);
	dx /= dxy;
	dy /= dxy;
	return VectorLine(m, c, dx, dy);
}

float vairanceOfPath(VectorLine sAxis, float velo, vector<pair<float, float>> pathHistory) {
	int len = pathHistory.size();
	float var = 0.0;
	for (int i = 1; i < len; i++) {
		float D = pathHistory[i].first - pathHistory[i - 1].first;
		if (D == 0)
			continue;
		float x = (pathHistory[i].second - pathHistory[i - 1].second) / D;
		float y = (
			pathHistory[i - 1].first * pathHistory[i].second -
			pathHistory[i].first * pathHistory[i - 1].second) / D;
		float distLine = (sAxis.a*x + sAxis.b*y + sAxis.cp)
			/ sqrt(sAxis.a*sAxis.a + sAxis.b*sAxis.b);
		if (distLine < 0) distLine *= (-1);
		var += distLine; // calculation like SSE :D
	}
	var = (var / len);
	var = ((int)var) % ((int)velo)*2.0 + 2.0; // averaging & biasing by the margin of 2
	return var; // var onk beshi ase - limit korte hbe
}

bool outsideOfParabola(float posX, float posY, BoundedParabola p)
{
	// Find whether on the same side of the bounding st. line as the focus
	float fVal = p.aBoundLine*p.focusX + p.bBoundLine*p.focusY + p.cBoundLine;
	float pVal = p.aBoundLine*posX + p.bBoundLine*posY + p.cBoundLine;
	if ((fVal > 0 && pVal < 0) || (fVal < 0 && pVal > 0) || pVal != 0)
		return true;
	float gVal = p.getPointval(posX, posY);
	float detVal = p.getDeterminantVal();

	if ((gVal < 0 && detVal < 0) || (gVal < 0 && detVal < 0) || gVal == 0)
		return false;
	return true;
}

bool outsideSafeRegion(float posX, float posY, BoundedParabola piR, float rSafe) {
	if (euclideanDist(piR.focusX, piR.focusY, posX, posY) > rSafe)
		return true;
	return outsideOfParabola(posX, posY, piR);
}

BoundedParabola getReliableRegion(BoundedParabola poiParabola,
	float mp, float np, float r) {
	// Need to change the directrix & bounding st. line of the poiParabola to get the reliable region's parabola

	// First, update the directrix
	float fx = poiParabola.focusX;
	float fy = poiParabola.focusY;

	float md = poiParabola.sDirtx.m;//-(poiParabola.aDirtx / poiParabola.bDirtx);
	float cd = poiParabola.sDirtx.c;// -(poiParabola.cDirtx / poiParabola.bDirtx);

	float ma = poiParabola.sAxis.m;
	float ca = poiParabola.sAxis.c;

	float dax = (cd - ca) / (ma - md);
	float day = (ma*cd - md*ca) / (ma - md);

	float a = mp*r;
	float rx = ((2 * a - r)*dax + r*fx) / (2 * a);
	float ry = ((2 * a - r)*day + r*fy) / (2 * a);
	float c = ry - md*rx;

	BoundedParabola relPbola = poiParabola;
	relPbola.aDirtx = md;
	relPbola.bDirtx = -1.0;
	relPbola.cDirtx = c;
	relPbola.sDirtx = VectorLine(md, c, 0.707, 0.707);

	// Now, update the bounding st. line
	float mb = md;
	float cb = -(poiParabola.cBoundLine / poiParabola.bBoundLine);

	float bax = (cb - ca) / (ma - mb);
	float bay = (ma*cb - mb*ca) / (ma - mb);
	float d = (np-1)*r;
	float bx = (d*bax + r*fx) / (np * r);
	float by = (d*bay + r*fy) / (np * r);
	cb = by - mb*bx;

	float focus[] = {fx, fy};
	relPbola.aBoundLine = mb;
	relPbola.bBoundLine = -1.0;
	relPbola.cBoundLine = cb;
	// relParabola = BoundedParabola(focus, poiParabola.sAxis, mp*r-r, np*r-r);

	float a2 = relPbola.aDirtx*relPbola.aDirtx;
	float b2 = relPbola.bDirtx*relPbola.bDirtx;
	relPbola.A = b2;
	relPbola.B = -2 * relPbola.aDirtx*relPbola.bDirtx;
	relPbola.C = a2;
	relPbola.D = -2 * relPbola.focusX  * (a2 + b2) - 2 * relPbola.cDirtx*relPbola.aDirtx;
	relPbola.E = -2 * relPbola.focusY * (a2 + b2) - 2 * relPbola.bDirtx*relPbola.cDirtx;
	relPbola.F = (relPbola.focusX*relPbola.focusX + relPbola.focusY*relPbola.focusY) * (a2 + b2) - relPbola.cDirtx*relPbola.cDirtx;

	return relPbola;
	// return BoundedParabola(focus, poiParabola.sAxis, mp*r - r, np*r - r);
}

// Algo 6
float UpdateOnLocationChange(float posX, float posY, float velo,
	// BoundedParabola piR, 
	float rSafe, vector<pair<float, float>> pathHistory) { //
	Stopwatch sw;
	sw.start();

	if (outsideOfParabola(posX, posY, relParabola)) {
		VectorLine sAxis = predictDirection(posX, posY, pathHistory);
		SAOSServer server;
		float q[] = { posX, posY };
		float mp = vairanceOfPath(sAxis, velo, pathHistory);
		float np = velo > 2.0 ? log(exp(1.0)*velo) : log(exp(1.0) + velo);
		np *= 5;

		curAnsSet = getUnifiedAnswerSet(*(server.getKnownRegionData(q, ALARM_RADIUS, sAxis,
			*curAnsSet, mp, np)));
		algoOccurs[4]++;

		rSafe = ConfigUpdate(posX, posY);
		algoOccurs[5]++;
		relParabola = getReliableRegion(curAnsSet->poiParabola, mp, np, ALARM_RADIUS);
	}
	else if (outsideSafeRegion(posX, posY, relParabola, rSafe)) {
		rSafe = ConfigUpdate(posX, posY);
		algoOccurs[5]++;
	}

	sw.stop();
	algoRunTimes[6] += sw.getDiff();
	algoOccurs[6]++;

	/*if (sAxis != nullptr) {
		FILE* fp = fopen("Client_LatestDirection.txt", "w");
		fprintf(fp, "%f %f", sAxis->dx, sAxis->dy);
		fclose(fp);
		delete sAxis;
	}*/

	return rSafe;
}

// Algo 3
void initClientSide(int clientId) {
	Stopwatch sw;
	sw.start();

	vector<pair<float, float>> cv = readClientFromInfoFile(clientId);

	int cid = (int)cv[0].first;
	float m = cv[0].second;
	float c = cv[1].first;
	float velo = cv[1].second;
	float x = cv[2].first;
	float y = cv[2].second;
	lastX = 10000;
	lastY = 10000;

	vector<pair<float, float>> pathHistory;
	int len = cv.size();
	for (int i = 3; i < len; i++)
		pathHistory.push_back(cv[i]);

	VectorLine sAxis = predictDirection(x, y, pathHistory);
	float mp = vairanceOfPath(sAxis, velo, pathHistory);
	float np = log(exp(1.0) + velo);

	float q[] = { x,y };
	SAOSServer server;
	/*curAnsSet = server.queryAlarmables(q, ALARM_RADIUS, sAxis,
		curAnsSet, mp, np);*/
	curAnsSet = getUnifiedAnswerSet(*(server.getKnownRegionData(q, ALARM_RADIUS, sAxis,
		*curAnsSet, mp, np)));
	algoOccurs[4]++;

	relParabola = getReliableRegion(curAnsSet->poiParabola, mp, np, ALARM_RADIUS);

	ConfigUpdate(x, y);
	algoOccurs[5]++;

	sw.stop();
	algoRunTimes[3] += sw.getDiff();
	algoOccurs[3]++;

	/*FILE* fp = fopen("Client_LatestDirection.txt", "w");
	fprintf(fp, "%f %f", sAxis.dx, sAxis.dy);
	fclose(fp);*/
}

void serverInit(float clientX, float clientY) {
	SAOSServer server;
	float clientPos[] = { clientX, clientY };
	if (curAnsSet == NULL || curAnsSet == nullptr)
		curAnsSet = new AnswerSet();
	if (AppConstants::ALGO_TYPE == AppConstants::NAIVE_ALGO) {
		Stopwatch sw;
		sw.start();
		curAnsSet = server.getAlarmables(clientPos, ALARM_RADIUS, *curAnsSet);
		sw.stop();
		algoRunTimes[1] += sw.getDiff();
		algoOccurs[1]++;
	}
	else {
		initClientSide(1);
	}
}
void Client::activateClient()
{
	// TO_DO Call server initialization
	// + TO_DO : If has any client init function, 
	//			then call it after getting the answer set from the server
	for (int i = 0; i < 11; i++) {
		algoOccurs[i] = algoOccurs[i] = 0;
		algoRunTimes[i] = algoRunTimes[i] = 0.0;
	}

	totalSession = 0.0;
	timeSpan.start();
	serverInit(this->clientLocation.longitude, this->clientLocation.lattitude);

	// Initiate movement
	writeClientInfoInFile(this->clientId, this->curM, this->curC, this->curVelocity,
		this->clientLocation.longitude, this->clientLocation.lattitude,
		this->pathHistoryVect);
	moveClient();
}

// Algo 2
void UpdateClient(float posX, float posY) {
	serverInit(posX, posY);
	int nPois = curAnsSet->distO_vect.size();
	for (int i = 0; i < nPois; i++)
		if (curAnsSet->distO_vect[i].distance < ALARM_RADIUS)
			doAlarmUser(curAnsSet->distO_vect[i].queryPoints);
	/*for (Obstacle* poi : curAnsSet->poiSet) {
		//vector<int> shortestPath;
			double dx = poi->vertices[0]->x;
			double dy = poi->vertices[0]->y;
		if ()
			doAlarmUser(poi);

			/*curAnsSet->visGraph->
				findShortestPath(posX, posY,
					poi->vertices[0]->x, poi->vertices[0]->y,
					1000, shortestPath
				)* /
			obstructedDistance(curAnsSet->visGraph, posX, posY, dx, dy)
				<= ALARM_RADIUS
	}
	/*double dx = curAnsSet->location.longitude - posX;
	double dy = curAnsSet->location.lattitude - posY;
	double locChangeEuc = sqrt(dx*dx + dy*dy);
	if (locChangeEuc > curAnsSet->minDistToUpdate) // For Naive, minDistToUpdate=0, so any movement calls the initialization function
		serverInit(posX, posY);*/
}

void timer_start(function<void(void)> func, unsigned int interval)
{
	std::thread([func, interval]() {
		while (true)
		{
			func();
			std::this_thread::sleep_for(std::chrono::milliseconds(interval));
		}
	}).detach();
}

void writeCurDataToFile(long double curTimeStamp) {
	// Write the current time-stamp data
	// Server-query
	FILE *fpNs = fopen("NaiveServerQ.csv", "a");
	fprintf(fpNs, "%lf,%ld\n", curTimeStamp, algoOccurs[1]);
	fclose(fpNs);

	FILE *fpMs = fopen("MainServerQ.csv", "a");
	fprintf(fpMs, "%lf,%ld\n", curTimeStamp, algoOccurs[4]);
	fclose(fpMs);

	// Run-time
	FILE *fpNr = fopen("NaiveRun.csv", "a");
	fprintf(fpNr, "%lf,%f\n", curTimeStamp, (algoRunTimes[1] + algoRunTimes[2]));
	fclose(fpNr);

	FILE *fpMr = fopen("MainRun.csv", "a");
	fprintf(fpMr, "%lf,%f\n", curTimeStamp, (
		algoRunTimes[3] + algoRunTimes[4] + algoRunTimes[5] + algoRunTimes[6]));
	fclose(fpMr);
}

bool do_something(int clientId)
{
	timeSpan.stop();
	totalSession += timeSpan.getDiff();
	writeCurDataToFile(totalSession);
	timeSpan.start();

	// cout << "Client " << clientId << " is doing something ..." << endl;

	// TODO Calc. trajectory & call update
	vector<pair<float, float>> cv = readClientFromInfoFile(clientId);
	int cid = (int)cv[0].first;
	float m = cv[0].second;
	float c = cv[1].first;
	float velo = cv[1].second;
	float x = cv[2].first;
	float y = cv[2].second;

	lastX = x;
	lastY = y;

	vector<pair<float, float>> pathHistory;
	int len = cv.size();
	for (int i = 3; i < len; i++)
		pathHistory.push_back(cv[i]);

	// Now calculate new position of the client according to the velocity & prev. direction
	double dist = (velo * 1000) * 2.0 / 3600.0; // velocty is in unit per hour (3600 seconds),
								//whereas this function is called per 2 seconds
	float x1 = x + 1.5;
	float y1 = (1.5 / m) + y;//m*x1 + c;
	float dy = (y1 - y);
	float dxy = sqrt(1.5*1.5 + dy*dy);
	float dx = 1.5 / dxy;
	dy = dy / dxy; // Now unit vector along the client's path is (dx, dy)
	x = x + dist * dx;
	y = y + dist * dy;
	/// TODO Later => Check whether this path has crossed any obstacle / threshold
	// If yes, then limit the path just before the obstacle
	// or, better - rotate the remainder path to any other direction
	if (x > X_UP_LIMIT) {
		x -= (2 * dist * dx);
		m = -1.0 / m; // 90 degree rotation about the previous direction
	}
	else if (x < 0.1) {
		x += (2 * dist * dx);
		m = -1.0 / m;
	}
	if (y > Y_UP_LIMIT) {
		y -= (2 * dist * dy);
		m = -1.0 / m;
	}
	else if (y < 0.1) {
		y += (2 * dist * dy);
		m = -1.0 / m;
	}

	// Velocity range : 0 (+2.xyz) ~ 40 (-2.xyz)
	if (rand() % 2)
		velo += ((rand() / RAND_MAX) + 2); // +2.xyz
	else
		velo -= ((rand() / RAND_MAX) + 2); // -2.xyz
	if (velo > 40)
		velo -= ((rand() / RAND_MAX) + 2); // -2.xyz
	if (velo < 0)
		velo += ((rand() / RAND_MAX) + 2); // +2.xyz

	// Coin toss - whether to change direction or not ;) 
	if (rand() % 2) {
		// calculate new direction params
		// -> y= mx + c => set of (m,c) pairs
		// m = m + random( -1.7321, 1.7321)  => m is allowed to be varied within -120 degrees to 120 degrees, except the case of restriction within [-10, 10]
		// c = y-mx, [current user location = (x,y)]
		double r = 2 * ((double)rand() / (RAND_MAX)) - (2.5 - 1.7321); // range : (-0.7679) ~ (1.2321)
		m += r;
		if (m > 10)
			m -= 10;
		else if (m < -10)
			m += 10;
		c = y - m*x;
	}
	pathHistory.push_back(make_pair(m, c));
	writeClientInfoInFile(cid, m, c, velo, x, y, pathHistory);

	if (AppConstants::ALGO_TYPE == AppConstants::NAIVE_ALGO) {
		// Naive : Run UpdateClient(q, A)
		Stopwatch sw;
		sw.start();
		UpdateClient(x, y);
		sw.stop();
		algoRunTimes[2] += sw.getDiff();
		algoOccurs[2]++;
	}
	else {
		// SAOS : Run ConfigUpdate(q, A) => Algo. 4
		rSafe = UpdateOnLocationChange(x, y, velo, rSafe, pathHistory);
	}
	writePerformanceParams();

	return true;
}

void Client::moveClient()
{
	auto ds = std::bind(&do_something, this->clientId);
	timer_start(ds, 2000);//std::bind(&, this->clientId)
}