const char *index_html = "<!DOCTYPE html>\
<html>\
<head>\
  <title>Arduino Web Server</title>\
  <script>\
    // Function to send \"hello\" to the Arduino's serial monitor\
    function sendHello() {\
      // Send the \"hello\" message to the server\
      const xhr = new XMLHttpRequest();\
      xhr.open('GET', '/send-hello'); // Assuming '/send-hello' is the route on the Arduino server\
      xhr.send();\
    }\
  </script>\
</head>\
<body>\
  <h1>Arduino Web Server</h1>\
  <button onclick=\"sendHello()\">Hello</button>\
</body>\
</html>";