package org.firstinspires.ftc.teamcode.adultbot.hardware.motors;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

public class Stoppers {
    /**
     * Stores all the Stopper objects it is given
     */
    private final Set<Stopper> stoppers = new HashSet<>();

    /**
     * create a Stoppers object with no stoppers in it yet
     */
    public Stoppers() {
    }

    /**
     * Create a Stoppers object from an existing collection of Stopper objects
     *
     * @param stoppers the collection of Stopper objects to copy
     */
    public Stoppers(Collection<Stopper> stoppers) {
        this.stoppers.addAll(stoppers);
    }

    public Stoppers(Stoppers stoppers) {
        this.stoppers.addAll(stoppers.stoppers);
    }

    /**
     * Add a Stopper to the list
     *
     * @param stopper the Stopper to add
     */
    public void add(Stopper stopper) {
        stoppers.add(stopper);
    }

    /**
     * Loop through all the stoppers and stop each one
     */
    public void stop() {
        for (Stopper stopper : stoppers) {
            stopper.stop();
        }
    }
}
