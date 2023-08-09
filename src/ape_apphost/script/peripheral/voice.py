#!/home/ape/tool/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import subprocess

def play_voice(location:str,name:str,type:str) -> None:
    """
    func: play voice on car.
    Params: 
    {
        "location":file location,  end with "/"
        "name": name of voice file.
        "type": file type of voice ,like  'mp3'.
        available file type:"8svx aif aifc aiff aiffc al amb amr-nb amr-wb anb au avr awb caf cdda cdr cvs cvsd cvu dat dvms f32 f4 f64 f8 fap flac fssd gsm gsrt hcom htk ima ircam la lpc lpc10 lu mat mat4 mat5 maud mp2 mp3 nist ogg paf prc pvf raw s1 s16 s2 s24 s3 s32 s4 s8 sb sd2 sds sf sl sln smp snd sndfile sndr sndt sou sox sph sw txw u1 u16 u2 u24 u3 u32 u4 u8 ub ul uw vms voc vorbis vox w64 wav wavpcm wv wve xa xi"
    }
    """
    available_file_type = "8svx aif aifc aiff aiffc al amb amr-nb amr-wb anb au avr awb caf cdda cdr cvs cvsd cvu dat dvms f32 f4 f64 f8 fap flac fssd gsm gsrt hcom htk ima ircam la lpc lpc10 lu mat mat4 mat5 maud mp2 mp3 nist ogg paf prc pvf raw s1 s16 s2 s24 s3 s32 s4 s8 sb sd2 sds sf sl sln smp snd sndfile sndr sndt sou sox sph sw txw u1 u16 u2 u24 u3 u32 u4 u8 ub ul uw vms voc vorbis vox w64 wav wavpcm wv wve xa xi"
    if(type not in available_file_type):
        raise("voice file type error")
    else:
        file = location + name + "." + type
        print("play voice file:  " + file)
        try:
            subprocess.Popen("play " + file, shell=True) # shell=True是非阻塞的关键
        except:
            raise("play voice command ERROR")

if __name__ == "__main__":
    play_voice("/home/ape/ape_-android-app/src/ape_apphost/script/peripheral/voicefile/","test","mp3")
    print("voice playing")
