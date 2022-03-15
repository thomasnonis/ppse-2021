//THIS CODE TURNS THE MONTH INTO THE NUMBER OF DAYS SINCE JANUARY 1ST.
//ITS ONLY PURPOSE IS FOR CALCULATING DELTA (DECLINATION), AND IS NOT USED IN THE HOUR ANGLE TABLE OR ANYWHERE ELSE.
      float daynum(float month){
       float day;
       if (month == 1){day=0;}
       if (month == 2){day=31;}       
       if (month == 3){day=59;}       
       if (month == 4){day=90;}
       if (month == 5){day=120;}
       if (month == 6){day=151;}
       if (month == 7){day=181;}
       if (month == 8){day=212;}
       if (month == 9){day=243;}
       if (month == 10){day=273;}
       if (month == 11){day=304;}
       if (month == 12){day=334;} 
       return day; 
      }

//THIS CODE TAKES THE DAY OF THE MONTH AND DOES ONE OF THREE THINGS: ADDS A DAY, SUBTRACTS A DAY, OR
//DOES NOTHING. THIS IS DONE SO THAT LESS VALUES ARE REQUIRED FOR THE NOON HOUR ANGLE TABLE BELOW.
       int dayToArrayNum(int day){
            if ((day == 1) || (day == 2) || (day == 3)){day = 0;}
            if ((day == 4) || (day == 5) || (day == 6)){day = 1;}  
            if ((day == 7) || (day == 8) || (day == 9)){day = 2;}
            if ((day == 10) || (day == 11) || (day == 12)){day = 3;}
            if ((day == 13) || (day == 14) || (day == 15)){day = 4;}
            if ((day == 16) || (day == 17) || (day == 18)){day = 5;}
            if ((day == 19) || (day == 20) || (day == 21)){day = 6;}
            if ((day == 22) || (day == 23) || (day == 24)){day = 7;}
            if ((day == 25) || (day == 26) || (day == 27)){day = 8;}
            if ((day == 28) || (day == 29) || (day == 30) || (day == 31)){day = 9;}
          return day;
       }

//////////////////////////////////////////////////////////////
//HERE IS THE TABLE OF NOON HOUR ANGLE VALUES. THESE VALUES GIVE THE HOUR ANGLE, IN DEGREES, OF THE SUN AT NOON (NOT SOLAR NOON)
//WHERE LONGITUDE = 0. DAYS ARE SKIPPED TO SAVE SPACE, WHICH IS WHY THERE ARE NOT 365 NUMBERS IN THIS TABLE.
      float FindH(int day, int month){
      float h;
      
      if (month == 1){
            float h_Array[10]={
            -1.038,-1.379,-1.703,-2.007,-2.289,-2.546,-2.776,-2.978,-3.151,-3.294,};
            h = h_Array[day];}

      if (month == 2){
            float h_Array[10]={
            -3.437,-3.508,-3.55,-3.561,-3.545,-3.501,-3.43,-3.336,-3.219,-3.081,};
            h = h_Array[day];}

      if (month == 3){
            float h_Array[10]={
            -2.924,-2.751,-2.563,-2.363,-2.153,-1.936,-1.713,-1.487,-1.26,-1.035,};
            h = h_Array[day];}

      if (month == 4){
            float h_Array[10]={
            -0.74,-0.527,-0.322,-0.127,0.055,0.224,0.376,0.512,0.63,0.728,};
            h = h_Array[day];}

      if (month == 5){
            float h_Array[10]={
            0.806,0.863,0.898,0.913,0.906,0.878,0.829,0.761,0.675,0.571,};
            h = h_Array[day];}

      if (month == 6){
            float h_Array[10]={
            0.41,0.275,0.128,-0.026,-0.186,-0.349,-0.512,-0.673,-0.829,-0.977,};
            h = h_Array[day];}
            
      if (month == 7){
            float h_Array[10]={
            -1.159,-1.281,-1.387,-1.477,-1.547,-1.598,-1.628,-1.636,-1.622,-1.585,};
            h = h_Array[day];}

      if (month == 8){
            float h_Array[10]={
            -1.525,-1.442,-1.338,-1.212,-1.065,-0.9,-0.716,-0.515,-0.299,-0.07,};
            h = h_Array[day];}

      if (month == 9){
            float h_Array[10]={
            0.253,0.506,0.766,1.03,1.298,1.565,1.831,2.092,2.347,2.593,};
            h = h_Array[day];}

      if (month == 10){
            float h_Array[10]={
            2.828,3.05,3.256,3.444,3.613,3.759,3.882,3.979,4.049,4.091,};
            h = h_Array[day];}

      if (month == 11){
            float h_Array[10]={
            4.1,4.071,4.01,3.918,3.794,3.638,3.452,3.236,2.992,2.722,};
            h = h_Array[day];}

      if (month == 12){
            float h_Array[10]={
            2.325,2.004,1.665,1.312,0.948,0.578,0.205,-0.167,-0.534,-0.893,};
            h = h_Array[day];}

return h;
      }
//////////////////////////////////////////////////////////////
