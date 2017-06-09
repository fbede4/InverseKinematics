#pragma once
#ifndef M_ROBOT_H
#define	M_ROBOT_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <cstdlib>
  
namespace M_Robot
{

#pragma region Mátrixok és vektorok	 

#define THRESHOLD 1e-5
#define ROUNDaTOb(a, b) ((a-b < THRESHOLD && a-b > -THRESHOLD)	? a = b : a)
#define NEARaTOb(a, b) (a-b < THRESHOLD && a-b > -THRESHOLD)


	class Matrix4x4;
	class Vector4;
	class Vector6;

	//rotációs mátrixból euler szögek számítása
	Vector6 r2eul(Matrix4x4& transformMatrix);
	//rotációs mátrixból euler szögek számítása
	Matrix4x4 eul2r(Vector6& posandrot);
	//-PI és PI közé teszi a vektor elemeit	k*PI eltolással ahol k egész
	void ValidateVector6(Vector6& vector);

	class Matrix4x4
	{
		friend Vector4;
		double matrix[16];

	public:
		Matrix4x4();
		void Transpose();	//transzponálja a mátrixot
		Matrix4x4 Transposed() const;	//a mátrix transzponáltját adja vissza
		double& operator()(const int row, const int col);
		Vector4 operator*(const Vector4& other) const;
		Matrix4x4 operator*(const Matrix4x4& other) const;
		bool operator==(Matrix4x4& other);
	};

	class Vector4
	{
		friend Matrix4x4;
		double vector[4];

	public:
		Vector4();
		double& operator()(int index);
		bool operator==(Vector4& other);
	};

	class Vector6
	{
		double vector[6];

	public:
		Vector6();
		double& operator()(int index);
		double DistanceSquare(Vector6& other);
		void SetValues(double value);
		void SetValues(double value0, double value1, double value2, double value3, double value4, double value5);
		void RadToDeg();
		void DegToRad();
		bool operator==(Vector6& other);
	};

	std::ostream& operator<<(std::ostream& os, Matrix4x4& matrix);
	std::ostream& operator<<(std::ostream& os, Vector4& vector);
	std::ostream& operator<<(std::ostream& os, Vector6& vector);

#pragma endregion

#pragma region Robotkar irányítása

	class Robot;
	class Joint;

	//egy csukló adatainak struktúrája
	struct Joint
	{
		double limit_neg;	//negatív limit
		double limit_pos;	//pozitív limit
		double a;			//a eltolás
		double d;			//d eltolás

	public:
		//beállítja a paramétereket
		void SetParams(double d, double a, double limit_neg, double limit_pos);

		//ha az állapot a határokon belül van igaz, egyébként hamis
		bool StateInBoundaries(double state);
	};

	//megvalósított mitsubishi robotkar
	//inverse és forward kinematika számolás
	class Robot
	{
		Joint joints[6];			//csuklók (határok, a és d eltolások)
		Matrix4x4 transformMatrix;	//adott csuklóállás mellett a transzformációs mátrix
		Vector6 posAndRot;			//adott csuklóállás mellett a végpont pozíciója és euler szögei
		Vector6 jointState[8];		//a robot végpontjának helyzetét eredményezõ négy állás
		bool validSolution[8];		//megmondja, hogy melyik indexû megoldás érvényes
		int solutionNumber;			//hányféleképpen érhetõ el a kért végpont, amennyi jointState érvényes
		Vector6 currentState;		//a robot jelenlegi állása, következõ állás meghatározásához ehhez keres közelit
		int currentSolutionIndex;	//a választott megoldás indexe a megoldások között


	private:
		//a mobilról küldött adatok alapján számol transfromMatrix-ot
		void MobileDataToMatrix(Vector6& vector);

		//kiszámolja a transzformációs mátrixból a végpont helyzetét és euler szögeit
		void CalculatePosAndRotFromTransformMatrix();

		//kiszámolja a transzformációs mátrixot a végpont helyzetébõl és euler szögeibõl
		void CalculateTransformMatrixFromPosAndRot();

		//elsõ csukló mind a négy megoldásra
		void CalculateJoint1(double x, double y);

		//2. és 3. csuklók solnum és solnum+1 megoldásainak kiszámolása
		void CalculateJoint23(double r, double z, int solNum);

		//utolsó három csukló az eddig lehetséges megoldásokra
		void CalculateJoint456(Matrix4x4& request);

		//megnézi, hogy érvényes-e a megoldás
		//ha nincs érvényes megoldás, hibát dob
		void CheckSolutionValidity();

		//megkeresi a legjobb megoldást
		void SearchBestSolution();

		Matrix4x4 TransformMatrix03(Vector6& request);
		Matrix4x4 TransformMatrix36(Vector6& request);
		Matrix4x4 TransformMatrix03_KR5like(Vector6& request);

	public:
		//csuklók paramétereinek beállítása
		Robot();

		Robot(bool is_KR5);

		//adott csuklóállások mellett kiszámolja a végpot helyzetét
		Matrix4x4 ForwardKinematic(Vector6& request);

		Matrix4x4 eul2r(Vector6& posandrot);

		//kiszámolja a csuklóállásokat a transzformációs mátrixból, amik a kért végpontot eredményezik
		//Az elõzõ álláshoz legközelebbi megoldást adja vissza
		Vector6 InverseKinematic(Matrix4x4& request);

		//kiszámolja a csuklóállásokat a végpontokból és euler szögekbõl, amik a kért végpontot eredményezik
		//Az elõzõ álláshoz legközelebbi megoldást adja vissza
		Vector6 InverseKinematic(Vector6& request);

		//matlab toolboxból implementált algoritmussal számol inverz kinematikát
		Vector6 ikine6s(Matrix4x4& request);

		//megmondja mennyi érvényes megoldása van
		int ValidSolutionNumber();

		//megmondja, hogy az adott indexû megoldás érvényes-e
		bool IsValidSolution(int index);

		//visszaadja a jelenlegi állapot indexét a megoldások között
		int GetCurrentSolutionIndex();

		//visszaadja az elõzõ állapothoz legközelebbi megoldást
		Vector6 GetIkineSolution();

		//visszaadja a kért indexû megoldást 
		Vector6 GetIkineSolution(int index);

		//visszaadja a transzformációs mátrixot
		Matrix4x4 GetTransformMatrix();

		//visszaadja a pozíciót és euler szögeket tartalmazó vektort
		Vector6 GetPositionAndRotation();

		//átállítja a csuklók állását a kért indexû megoldásra
		void ChangeState(int index);
	};	
#pragma endregion  

}	

#endif // !M_ROBOT_H   