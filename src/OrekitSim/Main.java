package OrekitSim;

import java.io.*;
import org.hipparchus.util.FastMath;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;

public class Main {

    public static void writeFile(String fileName, String line) {
        try {
            FileWriter fw = new FileWriter(fileName, true);
            fw.write(line);
            fw.close();
        } catch (IOException e) {
            System.out.println("Ooops... Wystąpił błąd: ");
            e.printStackTrace();
        }
    }


    public static void main(String[] args) {
        File orekitData = new File("src/orekit-data");
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));

        String line1 = "1 52492U 22049AT  22262.18422890  .00002810  00000-0  19455-3 0  9995";
        String line2 = "2 52492  53.2200 339.1713 0001564  50.8584 309.2547 15.08839596 21208";
        TLE PmTLE = new TLE(line1, line2);

        AbsoluteDate t0 = new AbsoluteDate(
                2022, 12, 24, 0, 0, 0.0, TimeScalesFactory.getUTC()
        );
        AbsoluteDate t1 = new AbsoluteDate(
                2022, 12, 24, 0, 3, 0.0, TimeScalesFactory.getUTC()
        );

        Multithreading object = new Multithreading("test.csv", "src/3le.txt", PmTLE, t0, t1);
        object.run();


    }
}