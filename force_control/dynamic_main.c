
#include "stdafx.h"
#include "DynamicModule.h"




void main1()
{
	DynamicMdl DynamicMod;
	DynamicMdl* p_dynm=&DynamicMod;
	initDynamicMdl(p_dynm);
	NewtonEuler(&p_dynm->Dyn, &p_dynm->dprf, &p_dynm->dprm);



	return;
}