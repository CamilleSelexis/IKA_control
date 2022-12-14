void homePage(EthernetClient *client_pntr){
  Serial.println("home");
  int state = getStatus();
  long current_time = millis();
  int seconds = (int) (current_time / 1000) % 60 ;
  int minutes = (int) ((current_time / (1000*60)) % 60);
  int hours   = (int) ((current_time / (1000*60*60)) % 24);
  char c[30];
  int l = sprintf(c, "%02d:%02d:%02d",hours,minutes,seconds);
  client_pntr->println(F("HTTP/1.1 200 OK"));
  client_pntr->println(F("Content-Type: text/html"));
  client_pntr->println(F("Connection: close"));  // the connection will be closed after completion of the response
  client_pntr->println(F("Refresh: 5"));  // refresh the page automatically every 5 sec
  client_pntr->println();
  client_pntr->println(F("<!DOCTYPE HTML>"));
  client_pntr->println(F("<html>"));
  client_pntr->println(F("<body>"));
  client_pntr->print("<h1 style=\"text-align:center\">IKA Controller  </h1>");
  client_pntr->print("<p> Heating : ");
  if(heatingEnable) client_pntr->println("<strong style= \"background-color:#00ff00\"> ON </strong>");
  else client_pntr->println("<strong style= \"background-color:#ff0000\"> OFF </strong>");
  client_pntr->print("</p> <p> Magnets : ");
  if(magnetEnable) client_pntr->println("<strong style= \"background-color:#00ff00\"> ON </strong> </p> ");
  else client_pntr->println("<strong style= \"background-color:#ff0000\"> OFF </strong> </p> ");
  String temp_below_color = "<strong style= \"background-color:#0000ff\">";
  String temp_above_color = "<strong style= \"background-color:#ff0000\">";
  client_pntr->print("<p> Current Temperature : ");
  if(temp<setpoint) client_pntr->print("<strong style= \"background-color:#0000ff\">");
  else client_pntr->print("<strong style= \"background-color:#ff0000\">");
  client_pntr->print(temp);
  client_pntr->print("</strong> Setpoint : ");client_pntr->print(setpoint);
  client_pntr->print(" Error : ");client_pntr->print(error);client_pntr->println("</p>");
  client_pntr->print("Current PID parameters : Kp = ");client_pntr->print(Kp);client_pntr->print(" Kd = ");client_pntr->print(Kd);
  client_pntr->print(" Ki = ");client_pntr->print(Ki);client_pntr->print(" Cycle Time = "); client_pntr->print(PID_TIME);
  client_pntr->println(F("<p><a href=\"http://192.168.1.91/HeatingON\">Enable Heating</a></p>"));
  client_pntr->println(F("<p><a href=\"http://192.168.1.91/HeatingOFF\">Disable Heating</a></p>"));
  client_pntr->println(F("<p><a href=\"http://192.168.1.91/MagnetsON\">Enable Magnets</a></p>"));
  client_pntr->println(F("<p><a href=\"http://192.168.1.91/MagnetsOFF\">Disable Magnets</a></p>"));
  client_pntr->println(F("<p><a href=\"http://192.168.1.91/ResetController\">Reset Controller</a></p>"));
  client_pntr->print("<p> Connection closed by the server at internal time : ");client_pntr->print(millis());
  //Close the connection
  client_pntr->print("</p> </body>");
  client_pntr->print("</html>");
  delay(10);
  //client_pntr->flush();
  while (client_pntr->read() != -1);
  ////Serial.println("Client stop called");
  client_pntr->stop();
}
void AnswerHttp(EthernetClient *client_pntr){
  client_pntr->println(F("HTTP/1.1 200 OK"));
  /*client_pntr->println(F("Content-Type: text/html"));
  client_pntr->println(F("Connection: close"));  // the connection will be closed after completion of the response
  client_pntr->println();*/
  client_pntr->print("status=");client_pntr->println(getStatus());
  client_pntr->print("temperature=");client_pntr->println(temp);
  client_pntr->print("time=");client_pntr->println(millis());

  //close the connection
  while (client_pntr->read() != -1);
  client_pntr->stop();
}
