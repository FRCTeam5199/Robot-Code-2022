package frc.misc;

import frc.robot.Main;

/**
 * Lets keep things fun and fresh by providing some of our best quotes.
 */
public class QuoteOfTheDay {
    /**
     * Hey these docs are cool but you wanna read the names from time to time? Please and thank you
     */
    public static final String[] quotes = {
            "Haha I'm simply vibing here.",
            "Joeys first time being cancelled, colorized, 2021.",
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "reeeeeeeeeeeeeeeeeeeeee",
            "I'm literally boiling",
            "Someone is legally obligated to hit it with a wrench and render it inoperable.",
            "It's what I like to call a gameing laptop. And by gameing I mean Joey's Awesome Boat Game.",
            "Sterling, you have what I like to call a not gameing laptop. You can't run Joey's Awesome Boat Game at 1,000 FPS.",
            "Aliiiiiiiiiii, why is the bot not working?",
            "Why does this sound so crap?", //Brandon's favorite
            "Me: Mom, can we get a random? \nMom: No, we have random at home. \nRandom at home:",
            "You mean to tell me its open source? hows it still *that* crap?",
            "your life savings? null",
            "Programmers probably don't live \nNot conventionally anyways",
            "I'm disappointed, but not surprised.",
            "I'm going to commit a hate crime.",
            "think of the emissions the planet would save if mojank optimized minecraft",
            "The best IDE for programming is Microsoft Word. It has the most customizable syntax hilighting you can get.",
            "You can also hide your code. If you have your API key exposed, you can just unexpose it like that.",
            "You see this auto capitalization? You might think that this is an inconvenience, but it's actually great",
            "It does that sometimes", //A close second - brandon
            "If you don't do that, it won't work. This is a feature, not a bug. It's, um, to prevent hackers from saving malicious code while they access your desktop.",
            "I'll start up Windows PowerShell because it is the best since Microsoft is the best and anything they make is the best.",
            "I understand what your saying... from a voodoo point of view.",
            "The bot started up without erroring, and I don't know why.",
            "Uh its not supposed to do that", // its a feature not a bug
            "im in a call with a psychopath \nthis is heinous \nits not slander if its true",
            "Why isn't it working? Oh wait, I'm not on the wifi.",
            "Why cant i connect? would help if the bot was on :/",
            "Hey, is the bot on? No? Well, it would help if it was on..",
            "It doesn't make any sense to you, and it doesn't make any sense to me.",
            "Did you delete it? \nYeah, I did. \nSo we both deleted it.. \nOh no. \nOh no..",
            "I can't believe it's butter.",
            "Boy, have I got the solution for you. I call it Accidental Dental.",
            "It shooby dooby do be like that.",
            "Why is it being criiiiinge???",
            "You said you would install it at the beginning of class. It's almost the end and it's still not on there.", // probably refrencing me
            "If you want someone to blame, talk to Teddy.",
            "Yo, Bougie Arouji",
            "Only 00:12? The night is still young.",
            "so when can we expect the break beams to be installed? \nGood question! If you would like to receive a response please press 1. If all your questions have been answered and you'd like to hear about what's new press 2. If you'd like to sign up for our email list press 3. If you wish to unsubscribe from our email list press 4. Good Bye!",
            "It's not working, and I don't know why.\nIt's working, and I don't know why.",
            "I did it! I plugged in the magic numbers.",
            "It may or may not work.",
            "Africa, Default Pi.. \nWhere's Africa? \n... Amica..",
            "Fix your software!!",
            "Oh no, it rickrolled me. Oh no! Kill the server!",
            "Sterling: I love my job! \nBrandon: That's a lie.",
            "Teddy, make your intake lighter.",
            "If in doubt, comment it out",
            "Sterling: Laugh less. Bu hao more.",
            "I can't connect to the robot if you don't turn it on...",
            "Sterling: Can the name on my hoodie be \"Pro 'grammer\"\nTeddy: I don’t think they can do thinking like ‘",
            "Sterling: Where's discipline? I need to teach the freshman a lesson.",
            "Teddy: Idk they obviously can’t read\nStryker: Bro you're so mean\nTeddy: Nvm he can read",
            "Sterling: Someone fried my RIO ports\nBrandon: Just like someone fried my teensy?",
            "Alex: I don't talk to the press",
            "Sterling: Well I didn't have the budget of 5.99 so I couldn't afford the MathematicalMod.",
            "s2: If there's chinese writing on it, it's bad",
            "Kyle: How can I live laugh love in these conditions?",
            "Rick: Teddy's just tall, that's it",
            "Ali: Try blowing on [the ethernet cable] (to fix it)",
            "Stryker: Considering my age and size I'm pretty bulked",
            "Sterling: Here in soviet programmer fun time land, you do not turn robot; robot turns you.",
            "Ah I'm bleeding, it's probably fine - everyone",
            "Sterling: You did it so well that you out human-played yourself.",
            "Alex: That's brutal"// that is brutal
    };

    /**

     *
     * @return a baller quote
     */
    public static String getRandomQuote() {
        return quotes[Main.RANDOM.nextInt(quotes.length)];
    }
}