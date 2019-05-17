//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "camera_index.h"
#include "Arduino.h"

#include "HTTPClient.h"
HTTPClient http;

#include "fb_gfx.h"
#include "fd_forward.h"
#include "dl_lib.h"
#include "fr_forward.h"

#include <EEPROM.h>
#define EEPROM_SIZE 25

#define ENROLL_CONFIRM_TIMES 5

#define COLOR_GREEN  0x0000FF00
#define COLOR_RED  0x00FF3034

boolean startup = true;

typedef struct {
        size_t size; //number of values used for filtering
        size_t index; //current value index
        size_t count; //value count
        int sum;
        int * values; //array to be filled with values
} ra_filter_t;

typedef struct {
        httpd_req_t *req;
        size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static ra_filter_t ra_filter;
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static mtmn_config_t mtmn_config = {0};
static int8_t detection_enabled = 1;

static ra_filter_t * ra_filter_init(ra_filter_t * filter, size_t sample_size){
    memset(filter, 0, sizeof(ra_filter_t));

    filter->values = (int *)malloc(sample_size * sizeof(int));
    if(!filter->values){
        return NULL;
    }
    memset(filter->values, 0, sample_size * sizeof(int));

    filter->size = sample_size;
    return filter;
}

static int ra_filter_run(ra_filter_t * filter, int value){
    if(!filter->values){
        return value;
    }
    filter->sum -= filter->values[filter->index];
    filter->values[filter->index] = value;
    filter->sum += filter->values[filter->index];
    filter->index++;
    filter->index = filter->index % filter->size;
    if (filter->count < filter->size) {
        filter->count++;
    }
    return filter->sum / filter->count;
}

static void rgb_print(dl_matrix3du_t *image_matrix, uint32_t color, const char * str){
    fb_data_t fb;
    fb.width = image_matrix->w;
    fb.height = image_matrix->h;
    fb.data = image_matrix->item;
    fb.bytes_per_pixel = 3;
    fb.format = FB_BGR888;
    fb_gfx_print(&fb, (fb.width - (strlen(str) * 14)) / 2, 10, color, str);
}

static int rgb_printf(dl_matrix3du_t *image_matrix, uint32_t color, const char *format, ...){
    char loc_buf[64];
    char * temp = loc_buf;
    int len;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
    va_end(copy);
    if(len >= sizeof(loc_buf)){
        temp = (char*)malloc(len+1);
        if(temp == NULL) {
            return 0;
        }
    }
    vsnprintf(temp, len+1, format, arg);
    va_end(arg);
    rgb_print(image_matrix, color, temp);
    if(len > 64){
        free(temp);
    }
    return len;
}


static void coffee_level(dl_matrix3du_t *image_matrix, boolean draw_only = true){
    //Serial.println("drawing line");

    int x, y, w, h, i;
    float send_value;
    float coffee_exists_level;
    int coffee_exists = 1;
    uint32_t color_green = COLOR_GREEN;
    uint32_t color_red = COLOR_RED;
    fb_data_t fb;
    fb.width = image_matrix->w;
    fb.height = image_matrix->h;
    fb.data = image_matrix->item;
    fb.bytes_per_pixel = 3;
    fb.format = FB_BGR888;

    int h_avg = 0;
    int h_last = 0;
    int h_values[fb.height]; 
    int min_line_y = abs((float)fb.height / 100 * EEPROM.read(24)/*Min*/ - fb.height);
    int max_line_y = abs((float)fb.height / 100 * EEPROM.read(25)/*Max*/ - fb.height);
    int left_line_x = (float)fb.width / 100 * EEPROM.read(26)/*Left*/;
    int right_line_x = (float)fb.width / 100 * EEPROM.read(27)/*Right*/;
    int coffee_exists_x = (float)fb.width / 100 * EEPROM.read(21)/*Coffee Exists X*/;
    int coffee_exists_y = (float)fb.height / 100 * EEPROM.read(22)/*Coffee Exists Y*/;
    int coffee_exists_threshold = EEPROM.read(8)/*Coffee Exists Threshold*/;

    int vflip = EEPROM.read(11)/*VFlip*/;
    
    for(int i = 0; i < fb.height; i++){

      h_avg = 0;
      
      for(int k = left_line_x*fb.bytes_per_pixel; k < fb.width*fb.bytes_per_pixel - (fb.width-right_line_x)*fb.bytes_per_pixel; k++){
        h_avg = h_avg + *(fb.data + i*fb.width*fb.bytes_per_pixel + k);
      }

      h_avg = int(h_avg / (fb.width*fb.bytes_per_pixel));

      if(EEPROM.read(19)/*Obscure*/ == true){
        for(int k = 0; k < fb.width*fb.bytes_per_pixel; k++){
          *(fb.data + i*fb.width*fb.bytes_per_pixel + k) = h_avg;
        }
      }
      
      h_values[i] = abs(h_avg - h_last);     
      h_last = h_avg;
      
    }

    int last_max = 0;
    int h_max = 0;
    for(int i = 5; i < fb.height - 5; i++){
      //Serial.println(h_values[i]);
      if(h_values[i] > last_max){
        h_max = i;
        last_max = h_values[i];
        
      }
      
    }
    
    send_value = abs(( (float)(h_max - max_line_y) / (float)(min_line_y - max_line_y) * 100) -100 );

    coffee_exists_level = *(fb.data + coffee_exists_y*fb.width*fb.bytes_per_pixel + coffee_exists_x*fb.bytes_per_pixel);

    if(coffee_exists_level > coffee_exists_threshold){
      coffee_exists = 0;
    }
    
    
    String str_send_value = (String)send_value;
    
    if(draw_only == true){
      // rectangle box
      x = 0;
      y = h_max;
      w = fb.width;
      h = fb.height;

      fb_gfx_drawFastHLine(&fb, x, min_line_y, w, color_green); //Min Line
      fb_gfx_drawFastHLine(&fb, x, max_line_y, w, color_green); //Max Line
      fb_gfx_drawFastVLine(&fb, left_line_x, 0, h, color_green); //Left Line
      fb_gfx_drawFastVLine(&fb, right_line_x, 0, h, color_green); //Right Line

      if(coffee_exists == 1){
        fb_gfx_fillRect(&fb, coffee_exists_x-2, coffee_exists_x, 3, 3, color_green);  //Coffee Pot Exists Marker
      }else{
        fb_gfx_fillRect(&fb, coffee_exists_x, coffee_exists_x, 3, 3, color_red);  //Coffee Pot Exists Marker
      }

      fb_gfx_fillRect(&fb, x, y, 30, 5, color_green);  //Coffee Level Marker

      if(EEPROM.read(18)/*Text*/ == true){
        fb_gfx_print(&fb, 40, y-10, color_green, &str_send_value[0]);
      }
      
    }else{
      //Serial.println( send_value );

      int httpCode = 0;
      http.begin("http://php-alnino200534546.codeanyapp.com/coffee/api.php?value=" + (String)send_value + "&cups=" + (String)EEPROM.read(17)/*Cups*/ + "&pot_id=" + (String)EEPROM.read(20)/*PotID*/ + "&exists=" + coffee_exists); //HTTP
      httpCode = http.GET(); 
      
      if (httpCode > 0) { //Check for the returning code
 
        String payload = http.getString();
        Serial.println("Response: " + (String)httpCode);
        Serial.println("Level: " + (String)payload);
        Serial.println("Cups: " + (String)EEPROM.read(17)/*Cups*/);
        Serial.println("Pot ID: " + (String)EEPROM.read(20)/*PotID*/);
        Serial.println("Coffee Exists: " + (String)coffee_exists);
 
      }else {
            Serial.println("Error on HTTP request");
      }

      http.end(); //Free the resources

    }

    

}



static size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len){
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if(!index){
        j->len = 0;
    }
    if(httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK){
        return 0;
    }
    j->len += len;
    return len;
}

static esp_err_t capture_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    int64_t fr_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    Serial.println(sizeof(fb->buf));

    if (!fb) {
        Serial.println("Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");

    size_t out_len, out_width, out_height;
    uint8_t * out_buf;
    bool s;
    if(fb->width > 400){
        size_t fb_len = 0;
        if(fb->format == PIXFORMAT_JPEG){
            fb_len = fb->len;
            res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
        } else {
            jpg_chunking_t jchunk = {req, 0};
            res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk)?ESP_OK:ESP_FAIL;
            httpd_resp_send_chunk(req, NULL, 0);
            fb_len = jchunk.len;
        }
        esp_camera_fb_return(fb);
        int64_t fr_end = esp_timer_get_time();
        Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start)/1000));
        return res;
    }

    dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
    if (!image_matrix) {
        esp_camera_fb_return(fb);
        Serial.println("dl_matrix3du_alloc failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    out_buf = image_matrix->item;
    out_len = fb->width * fb->height * 3;
    out_width = fb->width;
    out_height = fb->height;

    s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
    esp_camera_fb_return(fb);
    if(!s){
        dl_matrix3du_free(image_matrix);
        Serial.println("to rgb888 failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    jpg_chunking_t jchunk = {req, 0};
    s = fmt2jpg_cb(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, jpg_encode_stream, &jchunk);
    dl_matrix3du_free(image_matrix);
    if(!s){
        Serial.println("JPEG compression failed");
        return ESP_FAIL;
    }

    int64_t fr_end = esp_timer_get_time();
    return res;
}

void local_stream_handler(){
    camera_fb_t * fb = NULL;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;
    char * part_buf[64];
    dl_matrix3du_t *image_matrix = NULL;
    bool detected = false;
    
    int face_id = 0;
    int64_t fr_start = 0;
    int64_t fr_ready = 0;
    int64_t fr_face = 0;
    int64_t fr_recognize = 0;
    int64_t fr_encode = 0;

    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
    } else {
        fr_start = esp_timer_get_time();
        fr_ready = fr_start;
        fr_face = fr_start;
        fr_encode = fr_start;
        fr_recognize = fr_start;
        if(fb->width > 400){
            if(fb->format != PIXFORMAT_JPEG){
                bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                esp_camera_fb_return(fb);
                fb = NULL;
                if(!jpeg_converted){
                    Serial.println("JPEG compression failed");
                }
            } else {
                _jpg_buf_len = fb->len;
                _jpg_buf = fb->buf;
            }
        } else {
            image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);

            if (!image_matrix) {
                Serial.println("dl_matrix3du_alloc failed");
            } else {
                if(!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)){
                    Serial.println("fmt2rgb888 failed");
                } else {
                    
                    fr_ready = esp_timer_get_time();
                    //if (net_boxes || fb->format != PIXFORMAT_JPEG){
                    if (true || fb->format != PIXFORMAT_JPEG){
                    
                        coffee_level(image_matrix, false);

                        for(int i = 0; i < fb->height; i++){
                          for(int i = 0; i < fb->width * 3; i = i + 3){

                            *(image_matrix->item + i) = *(image_matrix->item + i) - 25;
                            
                          }
                        }                            

                        if(!fmt2jpg(image_matrix->item, fb->width*fb->height*3, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len)){
                            Serial.println("fmt2jpg failed");
                        }
                        
                        esp_camera_fb_return(fb);
                        fb = NULL;
                    } else {
                        _jpg_buf = fb->buf;
                        _jpg_buf_len = fb->len;
                    }
                    fr_encode = esp_timer_get_time();
                }
                dl_matrix3du_free(image_matrix);
            }
        }
    }
    
    if(fb){
        esp_camera_fb_return(fb);
        fb = NULL;
        _jpg_buf = NULL;
    } else if(_jpg_buf){
        free(_jpg_buf);
        _jpg_buf = NULL;
    }

    int64_t fr_end = esp_timer_get_time();

    int64_t ready_time = (fr_ready - fr_start)/1000;
    int64_t face_time = (fr_face - fr_ready)/1000;
    int64_t recognize_time = (fr_recognize - fr_face)/1000;
    int64_t encode_time = (fr_encode - fr_recognize)/1000;
    int64_t process_time = (fr_encode - fr_start)/1000;
    
    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;
    uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);

    

    last_frame = 0;
    return;
}

static esp_err_t stream_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;
    char * part_buf[64];
    dl_matrix3du_t *image_matrix = NULL;
    bool detected = false;
    int face_id = 0;
    int64_t fr_start = 0;
    int64_t fr_ready = 0;
    int64_t fr_face = 0;
    int64_t fr_recognize = 0;
    int64_t fr_encode = 0;

    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    while(true){

        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            res = ESP_FAIL;
        } else {
            fr_start = esp_timer_get_time();
            fr_ready = fr_start;
            fr_face = fr_start;
            fr_encode = fr_start;
            fr_recognize = fr_start;
            if(fb->width > 400){
                if(fb->format != PIXFORMAT_JPEG){
                    bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                    esp_camera_fb_return(fb);
                    fb = NULL;
                    if(!jpeg_converted){
                        Serial.println("JPEG compression failed");
                        res = ESP_FAIL;
                    }
                } else {
                    _jpg_buf_len = fb->len;
                    _jpg_buf = fb->buf;
                }
            } else {
                image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);

                if (!image_matrix) {
                    Serial.println("dl_matrix3du_alloc failed");
                    res = ESP_FAIL;
                } else {
                    if(!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)){
                        Serial.println("fmt2rgb888 failed");
                        res = ESP_FAIL;
                    } else {
                        
                        fr_ready = esp_timer_get_time();
                        //if (net_boxes || fb->format != PIXFORMAT_JPEG){
                        if (true || fb->format != PIXFORMAT_JPEG){
                            coffee_level(image_matrix);

                            for(int i = 0; i < fb->height; i++){
                              for(int i = 0; i < fb->width * 3; i = i + 3){
  
                                *(image_matrix->item + i) = *(image_matrix->item + i) - 25;
                                
                              }
                            }                            

                            if(!fmt2jpg(image_matrix->item, fb->width*fb->height*3, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len)){
                                Serial.println("fmt2jpg failed");
                                res = ESP_FAIL;
                            }
                            
                            esp_camera_fb_return(fb);
                            fb = NULL;
                        } else {
                            _jpg_buf = fb->buf;
                            _jpg_buf_len = fb->len;
                        }
                        fr_encode = esp_timer_get_time();
                    }
                    dl_matrix3du_free(image_matrix);
                }
            }
        }
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        
        if(fb){
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        } else if(_jpg_buf){
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if(res != ESP_OK){
            break;
        }
        int64_t fr_end = esp_timer_get_time();

        int64_t ready_time = (fr_ready - fr_start)/1000;
        int64_t face_time = (fr_face - fr_ready)/1000;
        int64_t recognize_time = (fr_recognize - fr_face)/1000;
        int64_t encode_time = (fr_encode - fr_recognize)/1000;
        int64_t process_time = (fr_encode - fr_start)/1000;
        
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);

    }

    last_frame = 0;
    return res;
}

static esp_err_t cmd_handler(httpd_req_t *req){
    
    char*  buf;
    size_t buf_len;
    char variable[32] = {0,};
    char value[32] = {0,};

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if(!buf){
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
                httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
            } else {
                free(buf);
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
        } else {
            free(buf);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        free(buf);
    } else {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    int val = atoi(value);
    sensor_t * s = esp_camera_sensor_get();

    int res = 0;
    if(!strcmp(variable, "framesize")) {
      if(s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
    }
    else if(!strcmp(variable, "quality")){
      EEPROM.write(1, val);
      res = s->set_quality(s, val);
    }
    else if(!strcmp(variable, "contrast")){
      EEPROM.write(2, val);
      res = s->set_contrast(s, val);
    }
    else if(!strcmp(variable, "brightness")){
      EEPROM.write(3, val);
      res = s->set_brightness(s, val);
    }
    else if(!strcmp(variable, "saturation")){
      EEPROM.write(4, val);
      res = s->set_saturation(s, val);
    }
    else if(!strcmp(variable, "gainceiling")){
      EEPROM.write(5, val);
      res = s->set_gainceiling(s, (gainceiling_t)val);
    }
    else if(!strcmp(variable, "colorbar")){
      EEPROM.write(6, val);
      res = s->set_colorbar(s, val);
    }
    else if(!strcmp(variable, "awb")){
      EEPROM.write(7, val);
      res = s->set_whitebal(s, val);
    }
    /*else if(!strcmp(variable, "agc")){
      EEPROM.write(8, val);
      res = s->set_gain_ctrl(s, val);
    }*/
    else if(!strcmp(variable, "aec")){
      EEPROM.write(9, val);
      res = s->set_exposure_ctrl(s, val);
    }
    else if(!strcmp(variable, "hmirror")){
      EEPROM.write(10, val);
      res = s->set_hmirror(s, val);
    }
    else if(!strcmp(variable, "vflip")){
      EEPROM.write(11, val);
      res = s->set_vflip(s, val);
    }
    else if(!strcmp(variable, "awb_gain")){
      EEPROM.write(12, val);
      res = s->set_awb_gain(s, val);
    }
    else if(!strcmp(variable, "agc_gain")){
      EEPROM.write(13, val);
      res = s->set_agc_gain(s, val);
    }
    else if(!strcmp(variable, "aec_value")){
      EEPROM.write(14, val);
      res = s->set_aec_value(s, val);
    }
    else if(!strcmp(variable, "aec2")){
      EEPROM.write(15, val);
      res = s->set_aec2(s, val);
    }
    else if(!strcmp(variable, "dcw")){
      EEPROM.write(16, val);
      res = s->set_dcw(s, val);
    }
    /*else if(!strcmp(variable, "bpc")){
      EEPROM.write(17, val);
      res = s->set_bpc(s, val);
    }
    else if(!strcmp(variable, "wpc")){
      EEPROM.write(18, val);
      res = s->set_wpc(s, val);
    }
    else if(!strcmp(variable, "raw_gma")){
      EEPROM.write(19, val);
      res = s->set_raw_gma(s, val);
    }
    else if(!strcmp(variable, "lenc")){
      EEPROM.write(20, val);
      res = s->set_lenc(s, val);
    }
    else if(!strcmp(variable, "special_effect")){
      EEPROM.write(21, val);
      res = s->set_special_effect(s, val);
    }
    else if(!strcmp(variable, "wb_mode")){
      EEPROM.write(22, val);
      res = s->set_wb_mode(s, val);
    }*/
    else if(!strcmp(variable, "ae_level")){
      EEPROM.write(23, val);
      res = s->set_ae_level(s, val);
    }

    //Coffee Settings
    else if(!strcmp(variable, "coffee_min")){
      EEPROM.write(24, val);
    }
    else if(!strcmp(variable, "coffee_max")){
      EEPROM.write(25, val);
    }
    else if(!strcmp(variable, "coffee_left")){
      EEPROM.write(26, val);
    }
    else if(!strcmp(variable, "coffee_right")){
      EEPROM.write(27, val);
    }
    else if(!strcmp(variable, "coffee_cups")){
      EEPROM.write(17, val);
    }
    else if(!strcmp(variable, "coffee_text")){
      EEPROM.write(18, val);
    }
    else if(!strcmp(variable, "coffee_obscure")){
      EEPROM.write(19, val);
    }
    else if(!strcmp(variable, "coffee_potid")){
      EEPROM.write(20, val);
    }
    else if(!strcmp(variable, "coffee_exists_x")){
      EEPROM.write(21, val);
    }
    else if(!strcmp(variable, "coffee_exists_y")){
      EEPROM.write(22, val);
    }
    else if(!strcmp(variable, "coffee_exists_threshold")){
      EEPROM.write(8, val);
    }
    /*else if(!strcmp(variable, "face_detect")) {
        detection_enabled = val;
        if(!detection_enabled) {
            recognition_enabled = 0;
        }
    } */   

    EEPROM.commit();
    
    if(res){
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t status_handler(httpd_req_t *req){
    static char json_response[1024];

    sensor_t * s = esp_camera_sensor_get();
    char * p = json_response;
    *p++ = '{';

    p+=sprintf(p, "\"framesize\":%u,", s->status.framesize);
    p+=sprintf(p, "\"quality\":%u,", s->status.quality);
    p+=sprintf(p, "\"brightness\":%d,", s->status.brightness);
    p+=sprintf(p, "\"contrast\":%d,", s->status.contrast);
    p+=sprintf(p, "\"saturation\":%d,", s->status.saturation);
    //p+=sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
    //p+=sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
    p+=sprintf(p, "\"awb\":%u,", s->status.awb);
    p+=sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
    p+=sprintf(p, "\"aec\":%u,", s->status.aec);
    p+=sprintf(p, "\"aec2\":%u,", s->status.aec2);
    p+=sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
    p+=sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
    //p+=sprintf(p, "\"agc\":%u,", s->status.agc);
    p+=sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
    p+=sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
    p+=sprintf(p, "\"bpc\":%u,", s->status.bpc);
    p+=sprintf(p, "\"wpc\":%u,", s->status.wpc);
    p+=sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
    p+=sprintf(p, "\"lenc\":%u,", s->status.lenc);
    p+=sprintf(p, "\"vflip\":%u,", s->status.vflip);
    p+=sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
    p+=sprintf(p, "\"dcw\":%u,", s->status.dcw);
    p+=sprintf(p, "\"colorbar\":%u,", s->status.colorbar);
    p+=sprintf(p, "\"coffee_min\":%u,",     EEPROM.read(24)/*Max*/);
    p+=sprintf(p, "\"coffee_max\":%u,",     EEPROM.read(25)/*Min*/);
    p+=sprintf(p, "\"coffee_left\":%u,",    EEPROM.read(26)/*Left*/);
    p+=sprintf(p, "\"coffee_right\":%u,",   EEPROM.read(27)/*Right*/);
    p+=sprintf(p, "\"coffee_cups\":%u,",    EEPROM.read(17)/*Cups*/);
    p+=sprintf(p, "\"coffee_text\":%u,",    EEPROM.read(18)/*Text*/);
    p+=sprintf(p, "\"coffee_obscure\":%u,", EEPROM.read(19)/*Obscure*/);
    p+=sprintf(p, "\"coffee_potid\":%u, ",    EEPROM.read(20)/*Pot ID*/);
    p+=sprintf(p, "\"coffee_exists_x\":%u, ", EEPROM.read(21)/*Coffee Exists X*/);
    p+=sprintf(p, "\"coffee_exists_y\":%u, ", EEPROM.read(22)/*Coffee Exists Y*/);
    p+=sprintf(p, "\"coffee_exists_threshold\":%u", EEPROM.read(8)/*Coffee Exists Threshold*/);
    *p++ = '}';
    *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t index_handler(httpd_req_t *req){
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    return httpd_resp_send(req, (const char *)index_html_gz, index_html_gz_len);
}

void startCameraServer(){
    EEPROM.begin(EEPROM_SIZE);
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t status_uri = {
        .uri       = "/status",
        .method    = HTTP_GET,
        .handler   = status_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t cmd_uri = {
        .uri       = "/control",
        .method    = HTTP_GET,
        .handler   = cmd_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t capture_uri = {
        .uri       = "/capture",
        .method    = HTTP_GET,
        .handler   = capture_handler,
        .user_ctx  = NULL
    };

   httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };


    ra_filter_init(&ra_filter, 20);
    
    mtmn_config.min_face = 80;
    mtmn_config.pyramid = 0.7;
    mtmn_config.p_threshold.score = 0.6;
    mtmn_config.p_threshold.nms = 0.7;
    mtmn_config.r_threshold.score = 0.7;
    mtmn_config.r_threshold.nms = 0.7;
    mtmn_config.r_threshold.candidate_number = 4;
    mtmn_config.o_threshold.score = 0.7;
    mtmn_config.o_threshold.nms = 0.4;
    mtmn_config.o_threshold.candidate_number = 1;
    
    
    Serial.printf("Starting web server on port: '%d'\n", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
        httpd_register_uri_handler(camera_httpd, &status_uri);
        httpd_register_uri_handler(camera_httpd, &capture_uri);
    }

    config.server_port += 1;
    config.ctrl_port += 1;
    Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}
