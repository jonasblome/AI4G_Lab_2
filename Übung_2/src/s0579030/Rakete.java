package s0579030;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Polygon;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Path2D.Float;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.PriorityQueue;
import java.util.Set;

import lenz.htw.ai4g.ai.AI;
import lenz.htw.ai4g.ai.DivingAction;
import lenz.htw.ai4g.ai.Info;
import lenz.htw.ai4g.ai.PlayerAction;

public class Rakete extends AI {
	Point[] pearls = info.getScene().getPearl(); // Pearls in current level
	float screenRatio = (float) (info.getScene().getWidth() * 1.0 / info.getScene().getHeight() * 1.0);
	int widthDivision = 150;
	int heightDivision = (int) (widthDivision * (1 / screenRatio)); // CHECK HERE IF ERROR
	Path2D[] obstacles = info.getScene().getObstacles(); // Obstacles in level
	Vertex[] vertices = new Vertex[widthDivision * heightDivision];
	Vector2D halfTileVector = new Vector2D(info.getScene().getWidth() / widthDivision / 2, info.getScene().getHeight() / heightDivision / 2);
	int pointsVisited = 0;
	ArrayList<Vector2D> smoothPath = new ArrayList<>();

	public Rakete(Info info) {
		super(info);
		enlistForTournament(579030, 577618);
		
		// Sort pearls by x value with bubble sort
		for(int i = 0; i < pearls.length - 1; i++) {
			for(int j = 0; j < pearls.length - i - 1; j++) {
				if(pearls[j].getX() > pearls[j + 1].getX()) {
					Point temp = pearls[j];
					pearls[j] = pearls[j + 1];
					pearls[j + 1] = temp;
				}
			}
		}
		
		// Set pearl position and starting position
		Vector2D pearlPosition = new Vector2D((float) pearls[0].getX(), (float) pearls[0].getY());
		Vector2D startingPosition = new Vector2D((float) info.getX(), (float) info.getY());
		
		// Scan grid for obstacles and set free tiles to true in freeSpace
		for(int x = 0; x < widthDivision; x++) {
			for(int y = 0; y < heightDivision; y++) {
				Rectangle2D currentTile = new Rectangle2D.Float();
				currentTile.setFrame(x * info.getScene().getWidth() / widthDivision, y * info.getScene().getHeight() / heightDivision, info.getScene().getWidth() / widthDivision, info.getScene().getHeight() / heightDivision);
				
				// Check each obstacle if it intersects with current tile
				for(int obstacle = 0; obstacle < obstacles.length; obstacle++) {
					if(obstacles[obstacle].intersects(currentTile)) {
						vertices[x + y * widthDivision] = null;
						break;
					}
					else {
						// If tile is free, create new vertex for graph in the middle of the tile
						Vector2D vertexPosition = new Vector2D((float) currentTile.getCenterX(), (float) currentTile.getCenterY());
						vertices[x + y * widthDivision] = new Vertex(vertexPosition, pearlPosition);
					}
				}
			}
		}
		
		// Set the neighbours for each vertex
		for(int vertex = 0; vertex < vertices.length; vertex++) {
			Vertex leftNeighbour = vertices[vertex];
			Vertex rightNeighbour = vertices[vertex];
			Vertex upperNeighbour = vertices[vertex];
			Vertex lowerNeighbour = vertices[vertex];
			
			// Only check for the vertices in freespace
			if(vertices[vertex] != null) {
				//links
				if(vertex % widthDivision == 0) {
					leftNeighbour = null;
				}
				//rechts
				if(vertex % widthDivision == widthDivision - 1) {
					rightNeighbour = null;
					
				}
				//oben
				if(vertex < widthDivision) {
					upperNeighbour = null;
					
				}
				//unten
				if(vertex > vertices.length - widthDivision - 1) {
					lowerNeighbour = null;
				}
				
				if(leftNeighbour == vertices[vertex]) {
					leftNeighbour = vertices[vertex - 1];
				}
				if(rightNeighbour == vertices[vertex]) {
					rightNeighbour = vertices[vertex + 1];
				}
				if(upperNeighbour == vertices[vertex]) {
					upperNeighbour = vertices[vertex - widthDivision];
				}
				if(lowerNeighbour == vertices[vertex]) {
					lowerNeighbour = vertices[vertex + widthDivision];
				}
				
				vertices[vertex].setNeighbour(0, leftNeighbour); //links
				vertices[vertex].setNeighbour(1, rightNeighbour); //rechts
				vertices[vertex].setNeighbour(2, upperNeighbour); //oben
				vertices[vertex].setNeighbour(3, lowerNeighbour); //unten
			}
		}
		
		// Retrieve level starting position
		startingPosition = new Vector2D((float) info.getX(), (float) info.getY());
		
		// Create path between pearl and smooth each section
		for(int i = 0; i < pearls.length; i++) {
			pearlPosition = new Vector2D((float) pearls[i].getX(), (float) pearls[i].getY());
			
			// Find shortest path to next pearl
			ArrayList<Vector2D> pathToNextPearl = aStarPathFinding(startingPosition, pearlPosition);
			
			// Add pearl position to current path segment and set starting position of path to current position
			pathToNextPearl.add(pearlPosition);
			pathToNextPearl.set(0, startingPosition);
			
			// Smooth the path from current position to next pearl
			smoothPath(pathToNextPearl);
			
			// Set starting position for next segment to current pearl position
			startingPosition = pearlPosition;
		}
		
		smoothPath.add(pearlPosition);
	}
	
	// Skip all vertices in path that are unnecessary
	public void smoothPath(ArrayList<Vector2D> path) {
		// Add the first vertex of the segment as a starting position
		smoothPath.add(path.get(0));
		
		// Check each vertex in segment if a line to it would intersect with the obstacles
		for(int i = 1; i < path.size(); i++) {
			// Creating a line between the last good path vertex and the current segment vertex
			Line2D lineBetweenVertices = new Line2D.Float();
			lineBetweenVertices.setLine(smoothPath.get(smoothPath.size()-1).getX(), smoothPath.get(smoothPath.size()-1).getY(), path.get(i).getX(), path.get(i).getY());
			
			// Check each obstacle if it intersects with the line
			for(Path2D obstacle : obstacles) {
				if(intersects(lineBetweenVertices, obstacle)) {
					// If they intersect, add the previous vertex to the smooth path and check a new line
					smoothPath.add(path.get(i-1));
					break;
				}
			}
		}
	}
	
	// Check if a line intersects with an obstacle
	public boolean intersects(Line2D line, Path2D path) {
		Point2D start = null;
		Point2D point1 = null;
		Point2D point2 = null;
		for (PathIterator pi = path.getPathIterator(null); !pi.isDone(); pi.next()) {
			float[] coordinates = new float[6];
			switch (pi.currentSegment(coordinates)) {
				case PathIterator.SEG_MOVETO:
					point2 = new Point2D.Float(coordinates[0], coordinates[1]);
					point1 = null;
					start = (Point2D) point2.clone();
					break;
				case PathIterator.SEG_LINETO:
					point1 = point2;
					point2 = new Point2D.Float(coordinates[0], coordinates[1]);
					break;
				case PathIterator.SEG_CLOSE:
					point1 = point2;
					point2 = start;
					break;
			}
			if (point1 != null) {
				Line2D segment = new Line2D.Float(point1, point2);
				if (segment.intersectsLine(line))
					return true;
			}
		}
		return false;
	}

	// Draw vertices and path
//	@Override
//	public void drawDebugStuff(Graphics2D gfx) {
//		for(int i = 0; i < smoothPath.size()-1; i++) {
//			gfx.setColor(Color.red);
//			gfx.drawLine((int) smoothPath.get(i).getX(), (int) smoothPath.get(i).getY(), (int) smoothPath.get(i+1).getX(), (int) smoothPath.get(i+1).getY());
//		}
//		
//		for(int i = 0; i < vertices.length; i++) {
//			if(vertices[i] != null) {
//				gfx.setColor(Color.pink);
//				gfx.drawOval((int) vertices[i].getLocation().getX(), (int) vertices[i].getLocation().getY(), 2, 2);
//			}
//		}
//	}
	
	// Returns a path from the closest vertex to the starting position to the closest vertex to the next pearl position
	public ArrayList<Vector2D> aStarPathFinding(Vector2D startingPosition, Vector2D pearlPosition) {
		// A star algorithm
		// Set all vertices of the graph to infinite distance with no previous node
		for(Vertex vertex: vertices) {
			if(vertex != null) {
				vertex.setDistanceFromStartPosition(Double.POSITIVE_INFINITY);
				vertex.setPreviousVertex(null);
				vertex.setExplored(false);
			}
		}
		
		// Add a queue for the next node with smallest distance
		Vertex currentVertex;
		PriorityQueue<Vertex> unexploredVertices = new PriorityQueue<Vertex>();
		
		// Find closest vertex to start position and to pearl position
		double startToVertexDistance = Double.POSITIVE_INFINITY;
		Vertex closestVertexToStart = vertices[0];
		
		double pearlToVertexDistance = Double.POSITIVE_INFINITY;
		Vertex closestVertexToPearl = vertices[0];
		
		// Check distance to start and pearl for each vertex
		for(int i = 0; i < vertices.length; i++) {
			Vertex vertexToMeasure = vertices[i];
			
			// Only check existing vertices
			if(vertexToMeasure != null) {
				double startDistanceToCurrentVertex = vertices[i].getLocation().subtractVector(startingPosition).getLength();
				double pearlDistanceToCurrentVertex = vertices[i].getLocation().subtractVector(pearlPosition).getLength();
				
				if(startDistanceToCurrentVertex < startToVertexDistance) {
					startToVertexDistance = startDistanceToCurrentVertex;
					closestVertexToStart = vertexToMeasure;
				}
				
				if(pearlDistanceToCurrentVertex < pearlToVertexDistance) {
					pearlToVertexDistance = pearlDistanceToCurrentVertex;
					closestVertexToPearl = vertexToMeasure;
				}
			}
		}
		
		// Add start node to the priority queue and set destination node
		unexploredVertices.add(closestVertexToStart);
		closestVertexToStart.setDistanceFromStartPosition(closestVertexToStart.getDistanceToEnd());
		closestVertexToStart.setPreviousVertex(null);
		Vertex destination = closestVertexToPearl;
		
		// For every neighbour of the current node update the distance
		// Set new previous node to current node
		while(!destination.getExplored()) {
			// Pull the nearest element out of the queue and get its neighbours
			currentVertex = (Vertex) unexploredVertices.poll();
			Vertex[] neighbours = new Vertex[4];
			
			if (currentVertex != null) {
				neighbours = currentVertex.getNeighbours();
			}
			else {
				return null;
			}
			
			// Look at all neighbours and check/update their distances
			for(Vertex neighbour : neighbours) {
				if(neighbour != null) {
					if (!neighbour.getExplored()) {
						// If the neighbour doesnt have a distance yet, set it and queue it
						if (neighbour.getDistanceFromStartPosition() == Double.POSITIVE_INFINITY) {
							neighbour.setDistanceFromStartPosition(currentVertex.getDistanceFromStartPosition() + 1 + currentVertex.getDistanceToEnd());
							neighbour.setPreviousVertex(currentVertex);
							unexploredVertices.add(neighbour);
						}
						// If it has a distance, just update it
						else {
							neighbour.setDistanceFromStartPosition(currentVertex.getDistanceFromStartPosition() + 1 + currentVertex.getDistanceToEnd());
							neighbour.setPreviousVertex(currentVertex);
						}
					}
				}
			}
			// Set current node to explored so it wont be checked again
			currentVertex.setExplored(true);
		}
		
		// Backtrack the path from the destination to the start and return it as string
		currentVertex = destination;
		ArrayList<Vector2D> path = new ArrayList<>();
		
		while(currentVertex != null) {
			path.add(currentVertex.getLocation());
			currentVertex = currentVertex.getPreviousVertex();
		}
		
		// Make path from start to pearl
		Collections.reverse(path);
		
		return path;
	}

	@Override
	public String getName() {
		return "Rakete";
	}

	@Override
	public Color getPrimaryColor() {
		return Color.CYAN;
	}

	@Override
	public Color getSecondaryColor() {
		return Color.BLUE;
	}

	@Override
	public PlayerAction update() {
		// Get diver position
		double startX = info.getX();
		double startY = info.getY();
		Vector2D startVector = new Vector2D((float) startX, (float) startY);
		
		// Get next point in path ArrayList
		double seekX = smoothPath.get(pointsVisited).getX();
		double seekY = smoothPath.get(pointsVisited).getY();
		Vector2D seekVector = new Vector2D((float) seekX, (float) seekY);
		
		// Check if point on path was visited
		if(Math.abs(startVector.getX() - seekVector.getX()) < 1 && Math.abs(startVector.getY() - seekVector.getY()) < 1) {
			pointsVisited++;
		}
		// Seek pearl
		Vector2D seekDirection = seekVector.subtractVector(startVector);
		seekDirection = seekDirection.normalize();
		
		// Calculate direction radiant value
		float direction = (float) Math.atan2(seekDirection.getY(), seekDirection.getX());
		return new DivingAction(1, -direction);
	}
}

