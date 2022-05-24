/*****************************************************************************/
/* File:        user.c (Khepera Simulator)                                   */
/* Author:      Olivier MICHEL <om@alto.unice.fr>                            */
/* Date:        Thu Sep 21 14:39:05 1995                                     */
/* Description: example of user.c file                                       */
/*                                                                           */
/* Copyright (c) 1995                                                        */
/* Olivier MICHEL                                                            */
/* MAGE team, i3S laboratory,                                                */
/* CNRS, University of Nice - Sophia Antipolis, FRANCE                       */
/*                                                                           */
/* Permission is hereby granted to copy this package for free distribution.  */
/* The author's name and this copyright notice must be included in any copy. */
/* Commercial use is forbidden.                                              */
/*****************************************************************************/

#include "../SRC/include.h"
#include "user_info.h"
#include "user.h"

#define FORWARD_SPEED   5                    /* normal (slow) forward speed*/
#define TURN_SPEED      4                    /* normal (slow) turn speed */
#define COLLISION_TH    900                  /* value of IR sensors to be 
                                                  considered as collision */

#define MAX(x, y) ((x>y)?x:y)
#define MIN(x, y) ((x<y)?x:y)

int pas=0;

typedef struct{

	float a;
	float b;
	float c;
	float d;

}Trapeze;

/**
 * 
 * Permet de trouver l'aire d'un trapèze à partir de quatre points et de sa hauteur
 * 
 **/

float Aire(Trapeze t, int hauteur){

	float triangle_gauche = (t.b - t.a) * (hauteur/2);
	float rectangle = (t.c - t.b) * hauteur;
	float triangle_droit = (t.d - t.c) * (hauteur/2);
	
	return triangle_gauche + rectangle + triangle_droit;

}

/**
 * 
 *Ce calcul est en fait celui du centre de gravité d'un trapèze, modifié pour coller aux exigences de l'exercice 
 *La version "officielle" de ce calcul s'appuie sur des distances et ne permet donc pas de trouver de résultat négatif
 *Ce qui était gênant dans le cas  
 * 
 **/

float Calcul_barycentre(Trapeze t, float hauteur){

	
	return (hauteur/3) * ((t.a+t.d)+(t.b+t.c)/4);	
	
}


float findY(Trapeze t, float x){

	if(x <= t.a || x >= t.d){
		return 0;	
	}
	else if(x >= t.b && x <= t.c){
		return 1;
	}
	else{
		if(x > t.a && x < t.b){
			return (1/(t.b - t.a)) * (x - t.a);
		}
		else{
			return (1/(t.d - t.c)) * (x - t.c);
		}
	}

}


float calculMoyennePonderee(float a1, float c1, float a2, float c2, float a3, float c3){
  return (a1*c1+a2*c2+a3*c3)/(a1+a2+a3);
}

void DrawStep()
{
  char text[256];

  sprintf(text,"step = %d",pas);
  Color(GREY);
  UndrawText(200,100,"step = 500");
  Color(BLUE);
  DrawText(200,100,text);
}

void UserInit(struct Robot *robot)
{
}

void UserClose(struct Robot *robot)
{
}

void NewRobot(struct Robot *robot)
{
  pas = 0;
}

void LoadRobot(struct Robot *robot,FILE *file)
{
}

void SaveRobot(struct Robot *robot,FILE *file)
{
}

void RunRobotStart(struct Robot *robot)
{
  ShowUserInfo(2,1);
}

void RunRobotStop(struct Robot *robot)
{
  ShowUserInfo(1,1);
}

boolean StepRobot(struct Robot *robot)
{
  pas++;
  DrawStep();
  
  Trapeze t_Loin, t_Proche, tTournerDroitemDroit, tTournerGauchemDroit, tTournerDroitemGauche, tTournerGauchemGauche, t_Avance;
  
  // Définition des trapèzes qui seront utilisés pour fuzzifier les données

  t_Loin.a = 0, t_Loin.b = 0, t_Loin.c = 6, t_Loin.d = 8;
  t_Proche.a = 4, t_Proche.b = 6, t_Proche.c = 1023, t_Proche.d = 1023;
  

  // Définition des trapèzes qui seront utilisés pour défuzzifier les données
  
  tTournerDroitemDroit.a = -5, tTournerDroitemDroit.b = -5, tTournerDroitemDroit.c = -1, tTournerDroitemDroit.d =-1;
  tTournerGauchemDroit.a = 0, tTournerGauchemDroit.b = 1, tTournerGauchemDroit.c = 5, tTournerGauchemDroit.d = 5;
  tTournerDroitemGauche.a = 1, tTournerDroitemGauche.b = 1, tTournerDroitemGauche.c = 5, tTournerDroitemGauche.d = 5;
  tTournerGauchemGauche.a = -5, tTournerGauchemGauche.b = -5, tTournerGauchemGauche.c = -1, tTournerGauchemGauche.d = 0;
  t_Avance.a = 0, t_Avance.b = 0, t_Avance.c = 5, t_Avance.d = 5;
 
  //Récupération des informations des capteurs, en faisant la moyenne des trois capteurs de chaque côté

  float moyenne_gauche = (robot->IRSensor[0].DistanceValue + robot->IRSensor[1].DistanceValue + robot->IRSensor[2].DistanceValue)/3;
  float moyenne_droite = (robot->IRSensor[3].DistanceValue + robot->IRSensor[4].DistanceValue + robot->IRSensor[5].DistanceValue)/3;
  
  //Fuzzification des données des capteurs
  
  float y_proche_gauche = findY(t_Proche, moyenne_gauche);
  float y_loin_gauche = findY(t_Loin, moyenne_gauche);
  float y_proche_droit = findY(t_Proche, moyenne_droite);
  float y_loin_droit = findY(t_Loin, moyenne_droite);
 
 //On trouve les hauteurs en fonction des règles définies
    //1. Si proche Gauche et loin Droit alors tourner à droite
  float hauteurR1 = MIN(y_proche_gauche, y_loin_droit);
    //2. Si proche Droit et loin Gauche alors tourner à gauche
  float hauteurR2 = MIN(y_proche_droit, y_loin_gauche);
    //3. Si loin Droit et loin Gauche alors avancer
  float hauteurR3 = MIN(y_loin_droit,y_loin_gauche);
    //4. Si proche Droit et proche Gauche alors tourner à gauche
  float hauteurR4 = MIN(y_proche_droit,y_proche_gauche);
 
  //Calcul des aires et des centres de gravités pour le côté gauche
  float aAvancerGauche = Aire(t_Avance, hauteurR3);
  float cAvancerGauche = Calcul_barycentre(t_Avance, hauteurR3);
  float aTournerDroiteGauche = Aire(tTournerDroitemGauche, hauteurR1);
  float cTournerDroiteGauche = Calcul_barycentre(tTournerDroitemGauche, hauteurR1);
  float aTournerGaucheGauche = (Aire(tTournerGauchemGauche, hauteurR2) + Aire(tTournerGauchemGauche, hauteurR4))/2; 
  float cTournerGaucheGauche = (Calcul_barycentre(tTournerGauchemGauche, hauteurR2) + Calcul_barycentre(tTournerGauchemGauche, hauteurR4))/2; 
  // Pour le côté droit
  float aAvancerDroit = Aire(t_Avance, hauteurR3); 
  float cAvancerDroit = Calcul_barycentre(t_Avance, hauteurR3); 
  float aTournerDroiteDroit = Aire(tTournerDroitemDroit, hauteurR1);
  float cTournerDroiteDroit = Calcul_barycentre(tTournerDroitemDroit, hauteurR1); 
  float aTournerGaucheDroit = (Aire(tTournerGauchemDroit, hauteurR2) + Aire(tTournerGauchemDroit, hauteurR4))/2;
  float cTournerGaucheDroit = (Calcul_barycentre(tTournerGauchemDroit, hauteurR2) + Calcul_barycentre(tTournerGauchemDroit, hauteurR4))/2;

  //Défuzzification des données et calcul de la vitesse pour chaque moteur
  float vitesseMoteurDroit = calculMoyennePonderee(aAvancerDroit,cAvancerDroit,aTournerDroiteDroit,cTournerDroiteDroit,aTournerGaucheDroit,cTournerGaucheDroit);
  float vitesseMoteurGauche =  calculMoyennePonderee(aAvancerGauche,cAvancerGauche,aTournerDroiteGauche,cTournerDroiteGauche,aTournerGaucheGauche,cTournerGaucheGauche);


  if ((aAvancerDroit+aTournerDroiteDroit+aTournerGaucheDroit == 0) || (aAvancerGauche+aTournerDroiteGauche+aTournerGaucheGauche==0)){
   //Afin d'éviter au robot de stagner
   robot->Motor[LEFT].Value  = FORWARD_SPEED;
   robot->Motor[RIGHT].Value = FORWARD_SPEED;
  }

  else{
    robot->Motor[LEFT].Value = vitesseMoteurGauche;
    robot->Motor[RIGHT].Value = vitesseMoteurDroit;
  }

  return(TRUE);

}


void FastStepRobot(struct Robot *robot)
{
}

void ResetRobot(struct Robot *robot)
{
  pas = 0;
}

void UserCommand(struct Robot *robot,char *text)
{
  WriteComment("unknown command"); /* no commands */
}

void DrawUserInfo(struct Robot *robot,u_char info,u_char page)
{
  char text[256];

  switch(info)
  {
    case 1:
      switch(page)
      {
        case 1: Color(MAGENTA);
                FillRectangle(0,0,40,40);
                Color(BLUE);
                DrawLine(100,100,160,180);
                Color(WHITE);
                DrawPoint(200,200);
                Color(YELLOW);
                DrawRectangle(240,100,80,40);
                Color(GREEN);
                DrawText(240,230,"hello world");
                break;
        case 2: Color(RED);
                DrawArc(200,50,100,100,0,360*64);
                Color(YELLOW);
                FillArc(225,75,50,50,0,360*64);
                Color(BLACK);
                DrawText(140,170,"This is the brain of the robot");
      }
      break;
    case 2:     DrawStep();
  }
}



