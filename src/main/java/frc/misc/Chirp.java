package frc.misc;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.discordslackbot.MessageHandler;
import frc.motors.TalonMotorController;
import frc.robot.Main;
import frc.robot.Robot;
import me.xdrop.fuzzywuzzy.FuzzySearch;

import java.io.File;
import java.util.*;

/**
 * This is where we play our gnarly tunez. Although it cant be seen from here, there is a delete deploy directory method
 * in {@link frc.robot.Robot} that will help remove ghost files. Use this like an {@link Orchestra} with extra steps
 */
@ClientSide
public class Chirp extends Orchestra implements ISubsystem {
    /**
     * Contains all of the talons created in {@link TalonMotorController#TalonMotorController(int)} that can be used to
     * play awesome tunez
     */
    public static final ArrayList<TalonMotorController> talonMotorArrayList = new ArrayList<>();
    /**
     * K: Title. V: All files matching (differs in motors required)
     */
    private static HashMap<String, List<String>> songnames;
    public static final SendableChooser<List<String>> MUSIC_SELECTION = getSongs();

    private final ArrayList<String> queue = new ArrayList<>();
    private boolean remoteCommand = false;

    /**
     * Loads up songs for {@link #songnames} and {@link UserInterface#MUSIC_SELECTOR}
     *
     * @return listObject a SendableChooser with all the songs
     */
    public static SendableChooser<List<String>> getSongs() {
        SendableChooser<List<String>> listObject = new SendableChooser<>();
        songnames = new HashMap<>();
        File[] files = Filesystem.getDeployDirectory().toPath().resolve("sounds").toFile().listFiles();
        for (File file : files) {
            String filename = file.getName().split("\\.")[0];
            try {
                Integer.parseInt(filename.split("_")[1]);
                Integer.parseInt(filename.split("_")[2]);
            } catch (Exception e) {
                continue;
            }
            if (!songnames.containsKey(filename.split("_")[0])) {
                songnames.put(filename.split("_")[0], new ArrayList<>(Collections.singleton(filename)));
            } else {
                songnames.get(filename.split("_")[0]).add(filename);
            }
        }
        List<String> filenames = new ArrayList<>();
        for (int i = 0; i < songnames.keySet().size(); i++)
            filenames.add((String) songnames.keySet().toArray()[i]);
        filenames.sort(String::compareTo);
        for (String key : filenames) {
            //System.out.println(key);
            listObject.setDefaultOption(key, songnames.get(key)); //WORKING: listObject.addOption(key, songnames.get(key));
        }
        return listObject;
    }

    public static String getAllSongs() {
        StringBuilder out = new StringBuilder("```\nSongs:");
        for (String name : songnames.keySet()) {
            out.append('\n').append(name);
        }
        return out.append("```").toString();
    }

    public Chirp() {
        init();
        addToMetaList();
    }

    /**
     * Add instruments and get ready to rumble
     */
    @Override
    public void init() {
        for (TalonMotorController motor : talonMotorArrayList) {
            motor.addToOrchestra(this);
        }
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return isPlaying() ? SubsystemStatus.NOMINAL : SubsystemStatus.FAILED;
    }

    /**
     * Autoplays music while taking suggestions. A few important things:
     * <p>
     * - Music names must be in format {@code <name>_<instruments>_<playtime in millis>.chrp}
     * <p>
     * - Songs will not be played if the {@link #talonMotorArrayList instrument registry} doesnt contain enough
     * instruments
     * <p>
     * - Autoplay will choose a new random song after the time specified in the file name
     */
    @Override
    public void updateTest() {
        if (remoteCommand || MessageHandler.canPersistCommands()) {
            handleRemoteUpdate();
        } else {
            handleLocalUpdate();
        }
    }

    private void handleRemoteUpdate() {
        //System.out.println("Running remote: " + queue.size());
        if (!isPlaying()) {
            if (queue.size() > 0) {
                loadMusic(queue.remove(0));
                play();
            } else {
                remoteCommand = false;
            }
        }
    }

    private void handleLocalUpdate() {
        List<String> selected = MUSIC_SELECTION.getSelected();
        String songName = "";//Robot.songTab.getString("");
        if (selected != null && selected.size() > 0) {
            for (String str : selected) {
                if (songName.equals("") && Integer.parseInt(str.split("_")[1]) <= talonMotorArrayList.size())
                    songName = str;
                else if (!songName.equals("") && Integer.parseInt(str.split("_")[1]) <= talonMotorArrayList.size() && Integer.parseInt(songName.split("_")[1]) < Integer.parseInt(str.split("_")[1]))
                    songName = str;
            }
        }
        UserInterface.MUSIC_FOUND_SONG.getEntry().setBoolean(new File(Filesystem.getDeployDirectory().toPath().resolve("sounds/" + songName + ".chrp").toString()).exists());
        if (!songName.equals("") && Filesystem.getDeployDirectory().toPath().resolve("sounds/" + songName + ".chrp").toFile().exists() && !songName.equals(Robot.lastFoundSong)) {
            Robot.lastFoundSong = songName;
            loadMusic(songName);
            ErrorCode e = play();
            if (e != ErrorCode.OK) {
                System.out.println("Music Error: " + e);
            }
            System.out.println("Playing song " + songName + " on " + Chirp.talonMotorArrayList.size() + " motors.");
        } else {
            if (!isPlaying()) {
                String randomSong = getRandomSong();
                System.out.println("Playing random song: " + randomSong);
                loadMusic(randomSong);
                play();
            }
        }

    }

    /**
     * Loads a song from the provided name assuming it is in the sounds deploy directory (deploy/sounds). Songs must be
     * in format {@code <name>_<instruments>_<playtime in millis>.chrp}
     *
     * @param soundName The name of the file (less extension, less path, just name) to get sound from (should be a .chrp
     *                  file)
     * @return Error code per super call
     */
    @Override
    public ErrorCode loadMusic(String soundName) {
        if (!Filesystem.getDeployDirectory().toPath().resolve("sounds/" + soundName + ".chrp").toFile().exists()) {
            if (songnames.containsKey(soundName)) {
                String songName = "";
                for (String str : songnames.get(soundName)) {
                    if (songName.equals("") && Integer.parseInt(str.split("_")[1]) <= talonMotorArrayList.size())
                        songName = str;
                    else if (!songName.equals("") && Integer.parseInt(str.split("_")[1]) <= talonMotorArrayList.size() && Integer.parseInt(songName.split("_")[1]) < Integer.parseInt(str.split("_")[1]))
                        songName = str;
                }
                super.loadMusic(Filesystem.getDeployDirectory().toPath().resolve("sounds/" + songName + ".chrp").toString());
                Robot.lastFoundSong = songName;
            }
        } else {
            Robot.lastFoundSong = soundName;
            ErrorCode e = super.loadMusic(Filesystem.getDeployDirectory().toPath().resolve("sounds/" + soundName + ".chrp").toString());
            if (e != ErrorCode.OK) {
                System.out.println("Failed to load " + Filesystem.getDeployDirectory().toPath().resolve("sounds/" + soundName + ".chrp") + ": " + e);
            }
            return e;
        }
        return ErrorCode.GeneralError;
    }

    /**
     * Overrides super to include if the playing song has ended as per given song length specified in file name
     *
     * @return if motors are in music mode and there is a song loaded that has yet to end
     */
    @Override
    public boolean isPlaying() {
        try {
            return super.isPlaying() && (!Robot.lastFoundSong.equals("") && getCurrentTime() < Integer.parseInt(Robot.lastFoundSong.split("_")[2]));
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Selects a random song name from options loaded at runtime in {@link Robot}
     *
     * @return song name from {@link #songnames}
     */
    public String getRandomSong() {
        String songName = "";
        for (String str : songnames.get(songnames.keySet().toArray()[Main.RANDOM.nextInt(songnames.keySet().toArray().length)])) {
            if (songName.equals("") && Integer.parseInt(str.split("_")[1]) <= talonMotorArrayList.size())
                songName = str;
            else if (!songName.equals("") && Integer.parseInt(str.split("_")[1]) <= talonMotorArrayList.size() && Integer.parseInt(songName.split("_")[1]) < Integer.parseInt(str.split("_")[1]))
                songName = str;
        }
        return songName;
    }

    @Override
    public void updateTeleop() {
        stop();
    }

    @Override
    public void updateAuton() {

    }

    @Override
    public void updateGeneric() {

    }

    @Override
    public void initTest() {

    }

    @Override
    public void initTeleop() {

    }

    @Override
    public void initAuton() {

    }

    @Override
    public void initDisabled() {

    }

    @Override
    public void initGeneric() {

    }

    @Override
    public String getSubsystemName() {
        return "Music";
    }

    public String playSongMostNearlyMatching(String songname) {
        if (!remoteCommand) {
            stop();
        }
        remoteCommand = true;
        if (songname.equals("rand") || songname.equals("random") || songname.equals("rng")) {
            String out = getRandomSong();
            if (isPlaying()) {
                queue.add(out);
            } else {
                loadMusic(out);
                play();
            }
            return out;
        }
        String winningSong = "";
        int winningScore = 0;
        for (String song : songnames.keySet()) {
            int thiscore = FuzzySearch.partialRatio(songname, song);
            if (thiscore > winningScore) {
                winningScore = thiscore;
                winningSong = song;
            }
        }
        if (winningScore < 50) {
            return "";
        }
        if (isPlaying()) {
            queue.add(winningSong);
        } else {
            loadMusic(winningSong);
            play();
        }
        return winningSong;
    }

    public int getQueueLength() {
        return queue.size();
    }

    public void skipSong() {
        if (queue.size() > 0) {
            queue.remove(0);
            stop();
        }
    }

    public String getQueueAsString() {
        if (queue.size() == 0) {
            if (isPlaying()) {
                return "playing " + Robot.lastFoundSong.split("_")[0];
            } else {
                return "nothing playing sob";
            }
        }
        StringBuilder out = new StringBuilder("```nim\n0) " + Robot.lastFoundSong.split("_")[0] + "\n");
        for (int i = 0; i < queue.size(); i++) {
            out.append(i + 1).append(") ").append(queue.get(i)).append('\n');
        }
        return out.append("```").toString();
    }
}