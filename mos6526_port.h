//////////////////////////////////////////////////
//						//
// Emu64                                        //
// von Thorsten Kattanek			//
//                                              //
// #file: mos6526_port.h                        //
//						//
// Dieser Sourcecode ist Copyright gesch�tzt!   //
// Geistiges Eigentum von Th.Kattanek		//
//						//
// Letzte �nderung am 12.04.2011		//
// www.emu64.de					//
//						//
//////////////////////////////////////////////////

#ifndef MOS_6526_PORT_H
#define MOS_6526_PORT_H

class PORT
{
public:
    PORT();
    ~PORT();
    unsigned char GetInput(void);
    void SetInput(unsigned char wert);
    unsigned char GetOutput(void);
    void SetOutput(unsigned char wert);
    unsigned char* GetOutputBitsPointer(void);
    unsigned char* GetInputBitsPointer(void);
    //bool SaveFreez(FILE* File);
    //bool LoadFreez(FILE* File,unsigned short Version);
private:
    unsigned char InputBits;
    unsigned char OutputBits;
};
#endif // MOS_6526_PORT_H
