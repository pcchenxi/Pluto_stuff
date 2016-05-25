#include <iostream>
#include <ncurses.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

 
int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt); //saves the current settings of the terminal as oldt.
  newt = oldt; //copy settings
  newt.c_lflag &= ~(ICANON | ECHO); //Goes through each line of newt and returns flag if it sees "\n", EOF or EOL
  tcsetattr(STDIN_FILENO, TCSANOW, &newt); //change settings of the terminal and saves them in newt
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0); //get file access mode and file status flags and save it in oldf
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK); //sets file status flags
 
  ch = getchar(); //get button pushed
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); //change setting to the old one
  fcntl(STDIN_FILENO, F_SETFL, oldf); //sets flags to the old ones
 
//Here the settings are back to the original ones and ch contains eitehr a button which was pressed or EOF
  //EOF means that ICANON went through the file and didn't find any flag

  if(ch != EOF) //if this isn't due to End of File.
  {
    ungetc(ch, stdin); //"undo getchar()", i.e. put the char back into the stream so that it can be found again.
    return 1;
  }
 
  return 0;
}

bool kbQuit()
{
  bool y=true;
  if (kbhit())
  {
    int c=getchar();
    switch(c)
    {
      case '\n':
        y=false; //stops doing the program
        //set speed to 0.
        std::cout << "User choose to quit. Speed is set to 0. " << std::endl;
        break;
      default:
        std::cout << "\n Press Enter if you wish to quit." << std::endl;
        break;
    }
  }
  return y;
}
