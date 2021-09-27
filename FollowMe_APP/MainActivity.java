package com.example.follow_me;

import android.Manifest;
import android.content.Intent;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.speech.tts.TextToSpeech;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Locale;

import static java.lang.Thread.sleep;

public class MainActivity extends AppCompatActivity implements TextToSpeech.OnInitListener {

    private TextToSpeech tts;
    Intent intent;
    SpeechRecognizer mRecognizer;
    TextView recieveText;
    EditText editTextAddress, editTextPort, messageText_s;
    EditText messageText_e;
    Button connectBtn, clearBtn;
    Button sttBtn_s;
    Button sttBtn_e;
    Integer who = 0;
    Integer clear = 0;
    Integer end = 0;
    Integer flag = 0;
    String address = "165.229.185.243";
    String port = "8080";
    final int PERMISSION = 1;

    Socket socket = null;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        if (Build.VERSION.SDK_INT >= 23) {
            //퍼미션 체크
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.INTERNET, Manifest.permission.RECORD_AUDIO}, PERMISSION);
        }
        //앱 기본 스타일 설정
        getSupportActionBar().setElevation(0);

        tts = new TextToSpeech(this,this);
        sttBtn_s = (Button) findViewById(R.id.sttStart_s);
        sttBtn_e = (Button) findViewById(R.id.sttStart_e);
        connectBtn = (Button) findViewById(R.id.buttonConnect);
        clearBtn = (Button) findViewById(R.id.buttonClear);
//        editTextAddress = (EditText) findViewById(R.id.addressText);
//        editTextPort = (EditText) findViewById(R.id.portText);
        recieveText = (TextView) findViewById(R.id.textViewReciev);
        messageText_s = (EditText) findViewById(R.id.startText);
        messageText_e = (EditText) findViewById(R.id.endText);
        intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
        intent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE, getPackageName());
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE,"ko-KR");


        sttBtn_s.setOnClickListener(v -> {
            who = 1;
            speakOut();
            mRecognizer=SpeechRecognizer.createSpeechRecognizer(this);
            mRecognizer.setRecognitionListener(listener);
            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            mRecognizer.startListening(intent);

        });

        sttBtn_e.setOnClickListener(v -> {
            who = 2;
            speakOut();
            end = 1;
            mRecognizer=SpeechRecognizer.createSpeechRecognizer(this);
            mRecognizer.setRecognitionListener(listener);
            try {
                sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            mRecognizer.startListening(intent);
        });

        //connect 버튼 클릭
        connectBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                MyClientTask myClientTask = new MyClientTask(address.toString(), Integer.parseInt(port.toString()), messageText_s.getText().toString(), messageText_e.getText().toString());
                myClientTask.execute();
                //messageText.setText("");
            }
        });

        //clear 버튼 클릭
        clearBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                recieveText.setText("");
                messageText_s.setText("");
                messageText_e.setText("");
            }
        });
    }

    public void moveActivity(String resultStr) {
        if(resultStr.indexOf("예") > -1) {
            MyClientTask myClientTask = new MyClientTask(address.toString(), Integer.parseInt(port.toString()), messageText_s.getText().toString(), messageText_e.getText().toString());
            myClientTask.execute();
        }
    }

    private void speakOut() {
        if(who == 1)
        {
            CharSequence text = messageText_s.getText();
            tts.setPitch((float) 1.0);
            tts.setSpeechRate((float) 0.8);
            tts.speak(text,TextToSpeech.QUEUE_FLUSH,null,"id1");
        }
        else if((who == 2) && (clear ==0))
        {
            CharSequence text = messageText_e.getText();
            tts.setPitch((float) 1.0);
            tts.setSpeechRate((float) 0.8);
            tts.speak(text,TextToSpeech.QUEUE_FLUSH,null,"id1");
            clear = 1;
        }
        else if((who == 2) && (clear == 1)&&(end == 1))
        {
            CharSequence text = "출발지는 " + messageText_s.getText() +"이고 도착지는 " +  messageText_e.getText() + "입니까?";
            tts.setPitch((float) 1.0);
            tts.setSpeechRate((float) 0.8);
            tts.speak(text,TextToSpeech.QUEUE_FLUSH,null,"id1");
            try {
                sleep(7000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            flag = 1;
            mRecognizer=SpeechRecognizer.createSpeechRecognizer(this);
            mRecognizer.setRecognitionListener(listener);
            mRecognizer.startListening(intent);
        }
    }


    @Override
    public void onDestroy() {
        if (tts != null) {
            tts.stop();
            tts.shutdown();
        } super.onDestroy();
    }


    private RecognitionListener listener = new RecognitionListener() {
        private int error;

        @Override public void onReadyForSpeech(Bundle params) {
            Toast.makeText(getApplicationContext(),"음성인식을 시작합니다.", Toast.LENGTH_SHORT).show();
        }

        @Override
        public void onBeginningOfSpeech() {}

        @Override
        public void onRmsChanged(float rmsdB) {}

        @Override
        public void onBufferReceived(byte[] buffer) {}

        @Override
        public void onEndOfSpeech() {}

        @Override
        public void onError(int error) {
            String message;

            switch (error) {
                case SpeechRecognizer.ERROR_AUDIO:
                    message = "오디오 에러";
                    break;

                case SpeechRecognizer.ERROR_CLIENT:
                    message = "클라이언트 에러";
                    break;

                case SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS:
                    message = "퍼미션 없음";
                    break;

                case SpeechRecognizer.ERROR_NETWORK:
                    message = "네트워크 에러";
                    break;

                case SpeechRecognizer.ERROR_NETWORK_TIMEOUT:
                    message = "네트웍 타임아웃";
                    break;

                case SpeechRecognizer.ERROR_NO_MATCH:
                    message = "찾을 수 없음";
                    break;

                case SpeechRecognizer.ERROR_RECOGNIZER_BUSY:
                    message = "RECOGNIZER가 바쁨";
                    break;

                case SpeechRecognizer.ERROR_SERVER:
                    message = "서버가 이상함";
                    break;

                case SpeechRecognizer.ERROR_SPEECH_TIMEOUT:
                    message = "말하는 시간초과";
                    break;

                default:
                    message = "알 수 없는 오류임";
                    break;
            }

            Toast.makeText(getApplicationContext(),"에러가 발생하였습니다. : " + message, Toast.LENGTH_SHORT).show();
        }
        @Override
        public void onResults(Bundle results) {
            // 말을 하면 ArrayList에 단어를 넣고 textView에 단어를 이어줍니다.
            ArrayList<String> matches =
                    results.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
            for(int i = 0; i < matches.size() ; i++){
                if(who == 1)
                {
                    messageText_s.setText(matches.get(i));
                }
                if((who == 2)&&(flag == 0))
                {
                    messageText_e.setText(matches.get(i));
                }
            }
            if(who == 1)
            {
                speakOut();
            }
            else if((who == 2)&&(flag == 0))
            {
                speakOut();
            }
            else if((who==2)&&(clear ==1)&&(end == 1))
            {
                String resultStr = "";
                for (int i = 0; i < matches.size(); i++) {
                    resultStr += matches.get(i);
                }
                moveActivity(resultStr);
            }
        }

        @Override public void onPartialResults(Bundle partialResults) {}
        @Override public void onEvent(int eventType, Bundle params) {}
    };

    @Override
    public void onInit(int status) {
        if (status == TextToSpeech.SUCCESS) {
            int result = tts.setLanguage(Locale.KOREA);

            if (result == TextToSpeech.LANG_MISSING_DATA || result == TextToSpeech.LANG_NOT_SUPPORTED) {
                Log.e("TTS", "This Language is not supported");
            } else {
                messageText_s.setEnabled(true);
                speakOut();
            }

        } else {
            Log.e("TTS", "Initilization Failed!");
        }
    }

    //
    public class MyClientTask extends AsyncTask<Void, Void, Void> {
        String dstAddress;
        int dstPort;
        String response = "";
        String myMessage = "";

        //constructor
        MyClientTask(String addr, int port, String message_s, String message_e){
            dstAddress = addr;
            dstPort = port;
            myMessage = message_s +"/"+ message_e + " app";
        }

        @Override
        protected Void doInBackground(Void... arg0) {

            Socket socket = null;
            myMessage = myMessage.toString();
            try {
                socket = new Socket(dstAddress, dstPort);
                //송신
                OutputStream out = socket.getOutputStream();
                out.write(myMessage.getBytes());

                //수신
                ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream(1024);
                byte[] buffer = new byte[1024];
                int bytesRead;
                InputStream inputStream = socket.getInputStream();
                /*
                 * notice:
                 * inputStream.read() will block if no data return
                 */
                while ((bytesRead = inputStream.read(buffer)) != -1){
                    byteArrayOutputStream.write(buffer, 0, bytesRead);
                    response += byteArrayOutputStream.toString("UTF-8");
                }
                response = "서버의 응답: " + response;
                socket.close();

            } catch (UnknownHostException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
                response = "UnknownHostException: " + e.toString();
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
                response = "IOException: " + e.toString();
            }finally{
                if(socket != null){
                    try {
                        socket.close();
                    } catch (IOException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                    }
                }
            }
            return null;
        }

        @Override
        protected void onPostExecute(Void result) {
            recieveText.setText(response);
            super.onPostExecute(result);
        }
    }

    private void onClick(View v) {
        mRecognizer = SpeechRecognizer.createSpeechRecognizer(this);
        mRecognizer.setRecognitionListener(listener);
        mRecognizer.startListening(this.intent);
    }

}