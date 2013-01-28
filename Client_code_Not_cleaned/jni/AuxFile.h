
#include <stdlib.h>
#ifndef AUXFILE_H
#define AUXFILE_H
//struct SCamInfo
class ScamInfo
{
public:
	float posx;
	float posy;
	float posz;
	float rotx;
	float roty;
	float rotz;
	float rotw;


	int pickedActGeodeNum;
	int pickedAreaNum;
	bool NewActGeode;

	//10 members so far
	bool startSendNode;
	unsigned int numBytesofNode;

	bool startSendMainScene;

	bool drawLine;
	bool newPt;
	float newPtx;
	float newPty;
	float newPtz;
	bool toggleStartDrawLine;
	//SCamInfo()

	bool toggleEditing;
	bool editing;
	bool deleteGeode;
	int deleteID;
	bool sendMarkers;

	bool addBox;
	float BoxX,BoxY,BoxZ;
	int BoxID;

	bool changeColor;
	float selfColorR,selfColorG,selfColorB;
	int changeColorID;

	ScamInfo()
	{
		posx=posy=posz=rotx=roty=rotz=rotw=0.0f;

		pickedActGeodeNum=0;
		pickedAreaNum=0;
		NewActGeode=false;

		sendMarkers=false;
		addBox=false;
		changeColor=false;
	}
};

#endif
