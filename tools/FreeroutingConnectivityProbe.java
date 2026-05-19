import app.freerouting.board.BasicBoard;
import app.freerouting.board.Component;
import app.freerouting.board.Item;
import app.freerouting.board.Pin;
import app.freerouting.board.Trace;
import app.freerouting.board.Via;
import app.freerouting.datastructures.UndoableObjects;
import app.freerouting.drc.AirLine;
import app.freerouting.interactive.RatsNest;
import app.freerouting.io.specctra.DsnReadResult;
import app.freerouting.io.specctra.DsnReader;
import app.freerouting.io.specctra.SesReader;
import app.freerouting.rules.Net;
import java.io.FileInputStream;
import java.nio.file.Path;
import java.util.Set;

/**
 * Probes freerouting's own connectivity model after importing a DSN and SES pair.
 *
 * <p>This is intentionally small and read-only: it helps compare the router's
 * ratsnest result with the exported KiCad geometry when a board reports fully
 * connected but the selector still sees missing terminal coverage.
 */
public final class FreeroutingConnectivityProbe {
  private FreeroutingConnectivityProbe() {
  }

  public static void main(String[] args) throws Exception {
    if (args.length != 2) {
      throw new IllegalArgumentException("usage: FreeroutingConnectivityProbe <dsn> <ses>");
    }

    Path dsnPath = Path.of(args[0]);
    Path sesPath = Path.of(args[1]);
    BasicBoard board = readBoard(dsnPath);
    try (FileInputStream input = new FileInputStream(sesPath.toFile())) {
      SesReader.read(input, board);
    }

    Net gnd = null;
    for (int netNo = 1; netNo <= board.rules.nets.max_net_no(); netNo++) {
      Net net = board.rules.nets.get(netNo);
      if (net != null && "GND".equals(net.name)) {
        gnd = net;
        break;
      }
    }
    if (gnd == null) {
      throw new IllegalStateException("GND net was not found");
    }

    RatsNest ratsNest = new RatsNest(board);
    System.out.println("gnd_net_no=" + gnd.net_number);
    System.out.println("ratsnest_total_incomplete=" + ratsNest.incomplete_count());
    System.out.println("ratsnest_gnd_incomplete=" + ratsNest.incomplete_count(gnd.net_number));
    System.out.println("gnd_pin_count=" + gnd.get_pins().size());
    for (AirLine airline : ratsNest.get_airlines()) {
      System.out.println("airline net=" + airline.net.net_number
          + " name=" + airline.net.name
          + " from=" + itemLabel(board, airline.from_item)
          + " from_corner=" + airline.from_corner
          + " from_contacts=" + airline.from_item.get_normal_contacts().size()
          + " from_connected_set=" + airline.from_item.get_connected_set(airline.net.net_number).size()
          + " to=" + itemLabel(board, airline.to_item)
          + " to_corner=" + airline.to_corner
          + " to_contacts=" + airline.to_item.get_normal_contacts().size()
          + " to_connected_set=" + airline.to_item.get_connected_set(airline.net.net_number).size());
      for (Item item : airline.from_item.get_connected_set(airline.net.net_number)) {
        System.out.println("  from_set_item=" + itemLabel(board, item)
            + " id=" + item.get_id_no()
            + " contacts=" + item.get_normal_contacts().size());
      }
      for (Item item : airline.to_item.get_connected_set(airline.net.net_number)) {
        System.out.println("  to_set_item=" + itemLabel(board, item)
            + " id=" + item.get_id_no()
            + " contacts=" + item.get_normal_contacts().size());
      }
    }

    Pin led3c = null;
    for (Pin pin : gnd.get_pins()) {
      Component component = board.components.get(pin.get_component_no());
      if (component != null && "LED3".equals(component.name) && "C".equals(pin.name())) {
        led3c = pin;
        break;
      }
    }
    if (led3c == null) {
      throw new IllegalStateException("LED3-C was not found on GND");
    }

    Set<Item> connected = led3c.get_connected_set(gnd.net_number);
    System.out.println("led3c_center=" + led3c.get_center());
    System.out.println("led3c_normal_contacts=" + led3c.get_normal_contacts().size());
    System.out.println("led3c_connected_set_size=" + connected.size());
    for (Item item : connected) {
      System.out.println("led3c_connected_item=" + item.getClass().getSimpleName()
          + " id=" + item.get_id_no()
          + " nets=" + item.net_count());
    }
  }

  private static BasicBoard readBoard(Path dsnPath) throws Exception {
    try (FileInputStream input = new FileInputStream(dsnPath.toFile())) {
      DsnReadResult result = DsnReader.readBoard(input, null, null, dsnPath.getFileName().toString());
      if (result instanceof DsnReadResult.Success success) {
        return success.board();
      }
      if (result instanceof DsnReadResult.OutlineMissing outlineMissing) {
        return outlineMissing.board();
      }
      throw new IllegalStateException("Failed to read DSN: " + result);
    }
  }

  private static String itemLabel(BasicBoard board, Item item) {
    if (item instanceof Pin pin) {
      Component component = board.components.get(pin.get_component_no());
      if (component != null) {
        return "Pin(" + component.name + "-" + pin.name() + ")";
      }
    }
    if (item instanceof Trace trace) {
      return "Trace(layer=" + trace.get_layer()
          + ", first=" + trace.first_corner()
          + ", last=" + trace.last_corner() + ")";
    }
    if (item instanceof Via via) {
      return "Via(first_layer=" + via.first_layer()
          + ", last_layer=" + via.last_layer()
          + ", center=" + via.get_center() + ")";
    }
    return item.getClass().getSimpleName() + "#" + item.get_id_no();
  }
}
