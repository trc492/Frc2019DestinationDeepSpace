/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package trclib;

import java.util.Locale;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * This class implements a generic request queue that runs on its own thread. It allows the caller to add requests
 * to the end of the queue. The request thread will perform the request asynchronously from the head of the queue.
 * When the request is completed, an optional event will be signaled.
 *
 * @param <R> specifies the type of the request context.
 */
public abstract class TrcRequestQueue<R>
{
    protected static final String moduleName = "TrcRequestQueue";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    /**
     * This interface is provided by the client to process the request at the head of the request queue. The
     * TrcRequestQueue class is request type agnostic. It does not understand the type of request nor does it
     * need to. Therefore, when the request is at the head of the queue and required processing, a client
     * provided process handler is called to process the request.
     *
     * @param <R> specifies the type of the request context.
     */
    public interface RequestHandler<R>
    {
        /**
         * This method is called to process a request.
         *
         * @param context specifies the request context.
         */
        public void processRequest(R context);
    }

    /**
     * This class implements a request. Typically, a request will be put into a FIFO request queue so that each
     * request will be processed in the order they came in.
     */
    public class Request
    {
        private R context;
        private RequestHandler<R> handler;
        private boolean repeat;
        private TrcEvent event;
        private boolean canceled;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param context specifies the request context.
         * @param handler specifies the request handler to call when the request is up for processing.
         * @param repeat specifies true to re-queue the request when completed.
         * @param event specifies the event to signal when the request is completed, can be null if none specified.
         */
        public Request(R context, RequestHandler<R> handler, boolean repeat, TrcEvent event)
        {
            this.context = context;
            this.handler = handler;
            this.repeat = repeat;
            this.event = event;
            this.canceled = false;
        }   //Request

        /**
         * This method checks if the request is canceled.
         *
         * @return true if the request is canceled, false otherwise.
         */
        public boolean isCanceled()
        {
            return canceled;
        }   //isCanceled

        /**
         * This method returns the request info as a string.
         *
         * @return request info string.
         */
        public String toString()
        {
            return String.format(Locale.US, "context=%s, repeat=%s, event=%s, canceled=%s",
                context, repeat, event, canceled);
        }   //toString

    }   //class Request

    private final String instanceName;
    private final LinkedBlockingQueue<Request> requestQueue;
    private volatile Thread requestThread = null;
    private boolean enabled = false;
    private Request priorityRequest = null;
    private TrcDbgTrace perfTracer = null;
    private double totalNanoTime = 0.0;
    private int totalRequests = 0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcRequestQueue(String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        requestQueue = new LinkedBlockingQueue<>();
    }   //TrcRequestQueue

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method enables/disables the request queue.
     *
     * @param enabled specifies true to enable request queue, false to disable.
     */
    public synchronized void setEnabled(boolean enabled)
    {
        if (enabled)
        {
            //
            // Enabling request queue, make sure the request queue is not already enabled.
            //
            if (requestThread == null)
            {
                requestThread = new Thread(this::requestTask, instanceName);
                requestThread.start();
                this.enabled = true;
            }
        }
        else if (requestThread != null)
        {
            //
            // Disabling request queue, make sure the request queue is indeed active.
            // The request queue may not be empty. So we need to signal termination but allow the request queue to
            // orderly shut down. If request queue is already disabled but the request thread is still active, it
            // means the thread is busy emptying its queue. So we don't need to double signal termination.
            //
            if (this.enabled)
            {
                this.enabled = false;
                requestThread.interrupt();
            }
        }
    }   //setEnabled

    /**
     * This method checks if the request queue is enabled.
     *
     * @return true if request queue is enabled, false if disabled.
     */
    public synchronized boolean isEnabled()
    {
        return enabled;
    }   //isEnabled

    /**
     * This method enables/disables performance report.
     *
     * @param tracer specifies the tracer to be used for performance tracing, can be null to disable performance
     *               tracing.
     */
    public synchronized void setPerformanceTracer(TrcDbgTrace tracer)
    {
        perfTracer = tracer;
    }   //setPerformanceTracer

    /**
     * This method queues a request at the end of the request queue to be processed asynchronously on a thread.
     *
     * @param context specifies the context of the request to be queued.
     * @param handler specifies the handler to call when the request is up for processing.
     * @param repeat specifies true to re-queue the request when completed.
     * @param event specifies the event to signal when the request is completed, can be null if none specified.
     * @return request entry added to the end of the queue. It can be used to cancel the request if it is still in
     *         queue.
     */
    public Request queueRequest(R context, RequestHandler<R> handler, boolean repeat, TrcEvent event)
    {
        final String funcName = "queueRequest";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "context=%s,repeat=%s,event=%s",
                context, repeat, event);
        }

        if (!isEnabled())
        {
            throw new RuntimeException("Request queue is not enabled, must call setEnabled first.");
        }

        Request request = new Request(context, handler, repeat, event);
        requestQueue.add(request);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", request);
        }

        return request;
    }   //queueRequest

    /**
     * This method executes the request synchronously. It adds the requst to the queue and wait for it to complete.
     *
     * @param context specifies the context of the request to be executed.
     * @param handler specifies the handler to call when the request is up for processing.
     * @return true if the request has been executed, false if it has been canceled.
     */
    public boolean executeRequest(R context, RequestHandler<R> handler)
    {
        final String funcName = "executeRequest";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "context=%s", context);
        }

        if (!isEnabled())
        {
            throw new RuntimeException("Request queue is not enabled, must call setEnabled first.");
        }

        TrcEvent event = new TrcEvent(instanceName + "." + funcName);
        Request request = queueRequest(context, handler, false, event);

        while (!event.isSignaled())
        {
            Thread.yield();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", !request.canceled);
        }

        return !request.canceled;
    }   //executeRequest

    /**
     * This method adds the priority request to the head of the queue. It will be processed once the current active
     * request is done processing.
     *
     * @param context specifies the context of the request to be executed.
     * @param handler specifies the handler to call when the request is up for processing.
     * @return request entry added to the head of the queue. It can be used to cancel the request if it is still in
     *         queue.
     */
    public synchronized Request queuePriorityRequest(R context, RequestHandler<R> handler)
    {
        final String funcName = "queuePriorityRequest";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "context=%s", context);
        }

        if (priorityRequest != null)
        {
            priorityRequest = new Request(context, handler, false, null);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "! (request=%s)", priorityRequest);
        }

        return priorityRequest;
    }   //queuePriorityRequest

    /**
     * This method cancels a request.
     *
     * @param request specifies the request entry from queueRequest or queuePriorityRequest to be canceled.
     * @return true if the request is found and canceled, false otherwise.
     */
    private synchronized boolean cancelRequest(Request request)
    {
        final String funcName = "cancelRequest";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "request=%s", request);
        }

        boolean foundRequest;
        if (request == priorityRequest)
        {
            priorityRequest = null;
            foundRequest = true;
        }
        else
        {
            foundRequest = requestQueue.contains(request);
        }

        if (foundRequest)
        {
            request.canceled = true;
            if (request.event != null)
            {
                request.event.set(true);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", foundRequest);
        }

        return foundRequest;
    }   //cancelRequest

    /**
     * This method is called when the request queue thread is started. It processes all requests in the request queue
     * when they arrive. If the request queue is empty, the thread is blocked until a new request arrives. Therefore,
     * this thread only runs when there are requests in the queue. If this thread is interrupted, it will clean up
     * the request queue before exiting.
     */
    private void requestTask()
    {
        final String funcName = "requestTask";
        Request request;

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "RequestQueue %s starting...", instanceName);
        }

        while (!Thread.currentThread().isInterrupted())
        {
            synchronized (this)
            {
                if (priorityRequest != null)
                {
                    request = priorityRequest;
                    priorityRequest = null;
                }
                else
                {
                    request = null;
                }
            }

            try
            {
                if (request == null)
                {
                    request = requestQueue.take();
                }

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "[%.3f] processing request %s", TrcUtil.getCurrentTime(), request);
                }

                long startNanoTime = TrcUtil.getCurrentTimeNanos();
                request.handler.processRequest(request.context);
                long elapsedTime = TrcUtil.getCurrentTimeNanos() - startNanoTime;
        
                totalNanoTime += elapsedTime;
                totalRequests++;
        
                if (perfTracer != null)
                {
                    perfTracer.traceInfo(funcName, "Average request process time = %.6f sec", totalNanoTime/totalRequests/1000000000.0);
                }
        
                if (request.event != null)
                {
                    request.event.set(true);
                }
        
                if (request.repeat)
                {
                    //
                    // This is a repeat request, add it back to the tail of the queue.
                    //
                    requestQueue.add(request);
                }
            }
            catch (InterruptedException e)
            {
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Terminating RequestQueue %s", instanceName);
                }
                break;
            }
        }
        //
        // The thread is terminating, empty the queue before exiting.
        //
        while ((request = requestQueue.poll()) != null)
        {
            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "[%.3f] Canceling request %s", TrcUtil.getCurrentTime(), request);
            }
            cancelRequest(request);
        }

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "RequestQueue %s is terminated", instanceName);
        }

        requestThread = null;
    }   //requestTask

}   //class TrcRequestQueue
