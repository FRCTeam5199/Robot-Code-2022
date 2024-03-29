package frc.discordslackbot.commands;

import com.slack.api.bolt.App;
import frc.discordslackbot.SlackBot;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;
import frc.robot.Robot;
import net.dv8tion.jda.api.EmbedBuilder;
import net.dv8tion.jda.api.JDA;
import org.jetbrains.annotations.NotNull;

/**
 * Returns a pretty listing of all activates subsystems and if they are working nominally
 */
public class StatusCommand extends AbstractCommand {
    @Override
    public @NotNull AbstractCommandResponse run(AbstractCommandData message) {
        StringBuilder statuses = new StringBuilder("```diff\n");
        for (ISubsystem system : Robot.subsystems)
            statuses.append(system.getSubsystemStatus() == SubsystemStatus.FAILED ? "- " : "+ ").append(system.getSubsystemName()).append(": ").append(system.getSubsystemStatus().name()).append('\n');
        statuses.append("```");
        return new StatusCommandResponse(message, "Activated subsystem statuses:", "jojo2357", statuses.toString());
    }

    @Override
    public String getCommand() {
        return "status";
    }

    @Override
    public String sendHelp() {
        return "Returns a list of all the active subsystems and their status.";
    }

    @Override
    public String[] getAliases() {
        return new String[]{"stat"};
    }

    /**
     * Response requires an embed and this embed is held in pieces at {@link #TITLE}, {@link #AUTHOR}, and {@link
     * #REPLY_CONTENT}
     */
    public static class StatusCommandResponse extends AbstractCommandResponse {
        private final String TITLE, AUTHOR, REPLY_CONTENT;

        public StatusCommandResponse(AbstractCommandData data, String title, String author, String reply_content) {
            super(data);
            TITLE = title;
            AUTHOR = author;
            REPLY_CONTENT = reply_content;
        }

        @Override
        public void doYourWorst(JDA client) {
            EmbedBuilder builder = new EmbedBuilder();
            builder.setTitle(TITLE).setDescription(REPLY_CONTENT).setAuthor(AUTHOR);
            client.getTextChannelById(CHANNEL_ID).sendMessage(builder.build()).submit();
        }

        @Override
        public void doYourWorst(App client) {
            SlackBot.sendSlackMessage(CHANNEL_ID, REPLY_CONTENT);
        }

        @Override
        public void doYourWorst() {
            System.out.println(REPLY_CONTENT);
        }
    }
}
